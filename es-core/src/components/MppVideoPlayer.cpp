#ifdef HAVE_MPP

#include "components/MppVideoPlayer.h"
#include "components/VideoVlcComponent.h"
#include "Log.h"

#include <chrono>
#include <vector>
#include <unistd.h>
#include <SDL.h>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavcodec/bsf.h>
#include <libavutil/channel_layout.h>
#include <libswscale/swscale.h>
#include <libswresample/swresample.h>
}

#define MODULE_TAG "es_video"	// tags MPP buffer allocations
#include <rockchip/rk_mpi.h>

#define MPP_DEVICE		"/dev/mpp_service"
#define FRAME_BUFFERS	20		// H.264 DPB (16) + frames in flight
#define LATE_DROP_MS	100		// frames later than this are skipped instead of shown

static long long nowMs()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool MppVideoPlayer::isAvailable()
{
	return access(MPP_DEVICE, R_OK | W_OK) == 0;
}

MppVideoPlayer* MppVideoPlayer::open(const std::string& path)
{
	if (!isAvailable())
		return nullptr;

	MppVideoPlayer* player = new MppVideoPlayer();
	if (!player->openFile(path) || !player->initDecoder())
	{
		delete player;
		return nullptr;
	}

	return player;
}

MppVideoPlayer::MppVideoPlayer() :
	mFormat(nullptr), mBsf(nullptr), mAudioCodec(nullptr), mPacket(nullptr), mAudioFrame(nullptr),
	mSws(nullptr), mSwr(nullptr), mVideoStream(-1), mAudioStream(-1), mWidth(0), mHeight(0),
	mMpp(nullptr), mMpi(nullptr), mFrameGroup(nullptr), mFrameEos(false),
	mAudioDevice(0), mWithAudio(false), mContext(nullptr), mOutWidth(0), mOutHeight(0), mStartTime(0),
	mClockStart(-1), mFirstPts(0), mStop(false), mEnded(false), mRestart(false)
{
}

MppVideoPlayer::~MppVideoPlayer()
{
	{
		std::unique_lock<std::mutex> lock(mLock);
		mStop = true;
	}
	mWake.notify_all();

	if (mThread.joinable())
		mThread.join();

	if (mAudioDevice)
		SDL_CloseAudioDevice(mAudioDevice);

	if (mMpp)
		mpp_destroy((MppCtx)mMpp);

	if (mFrameGroup)
		mpp_buffer_group_put((MppBufferGroup)mFrameGroup);

	sws_freeContext(mSws);
	swr_free(&mSwr);
	av_frame_free(&mAudioFrame);
	av_packet_free(&mPacket);
	avcodec_free_context(&mAudioCodec);
	av_bsf_free(&mBsf);
	avformat_close_input(&mFormat);
}

bool MppVideoPlayer::openFile(const std::string& path)
{
	if (avformat_open_input(&mFormat, path.c_str(), nullptr, nullptr) < 0)
		return false;

	if (avformat_find_stream_info(mFormat, nullptr) < 0)
		return false;

	mVideoStream = av_find_best_stream(mFormat, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
	if (mVideoStream < 0)
		return false;

	AVCodecParameters* par = mFormat->streams[mVideoStream]->codecpar;
	if (par->codec_id != AV_CODEC_ID_H264 || par->width <= 0 || par->height <= 0)
		return false;

	mWidth = par->width;
	mHeight = par->height;
	mAudioStream = av_find_best_stream(mFormat, AVMEDIA_TYPE_AUDIO, -1, mVideoStream, nullptr, 0);

	// MPP wants Annex-B start codes; MP4 stores length-prefixed NAL units
	const AVBitStreamFilter* filter = av_bsf_get_by_name("h264_mp4toannexb");
	if (!filter || av_bsf_alloc(filter, &mBsf) < 0)
		return false;

	avcodec_parameters_copy(mBsf->par_in, par);
	mBsf->time_base_in = mFormat->streams[mVideoStream]->time_base;
	if (av_bsf_init(mBsf) < 0)
		return false;

	mPacket = av_packet_alloc();
	return mPacket != nullptr;
}

bool MppVideoPlayer::initDecoder()
{
	MppCtx ctx = nullptr;
	MppApi* mpi = nullptr;

	if (mpp_create(&ctx, &mpi) != MPP_OK)
		return false;

	mMpp = ctx;
	mMpi = mpi;

	// packets are whole access units from the demuxer, no parsing needed
	RK_U32 split = 0;
	mpi->control(ctx, MPP_DEC_SET_PARSER_SPLIT_MODE, &split);

	if (mpp_init(ctx, MPP_CTX_DEC, MPP_VIDEO_CodingAVC) != MPP_OK)
	{
		LOG(LogWarning) << "MppVideoPlayer: mpp_init failed, falling back to VLC";
		return false;
	}

	return true;
}

bool MppVideoPlayer::initAudio()
{
	AVCodecParameters* par = mFormat->streams[mAudioStream]->codecpar;
	const AVCodec* codec = avcodec_find_decoder(par->codec_id);
	if (!codec)
		return false;

	mAudioCodec = avcodec_alloc_context3(codec);
	if (!mAudioCodec || avcodec_parameters_to_context(mAudioCodec, par) < 0 || avcodec_open2(mAudioCodec, codec, nullptr) < 0)
		return false;

	SDL_AudioSpec want, have;
	SDL_zero(want);
	want.freq = mAudioCodec->sample_rate > 0 ? mAudioCodec->sample_rate : 44100;
	want.format = AUDIO_S16SYS;
	want.channels = 2;
	want.samples = 1024;

	mAudioDevice = SDL_OpenAudioDevice(nullptr, 0, &want, &have, SDL_AUDIO_ALLOW_FREQUENCY_CHANGE);
	if (!mAudioDevice)
	{
		LOG(LogWarning) << "MppVideoPlayer: no audio device: " << SDL_GetError();
		return false;
	}

	AVChannelLayout stereo = AV_CHANNEL_LAYOUT_STEREO;
	if (swr_alloc_set_opts2(&mSwr, &stereo, AV_SAMPLE_FMT_S16, have.freq,
		&mAudioCodec->ch_layout, mAudioCodec->sample_fmt, mAudioCodec->sample_rate, 0, nullptr) < 0 || swr_init(mSwr) < 0)
		return false;

	mAudioFrame = av_frame_alloc();
	return mAudioFrame != nullptr;
}

void MppVideoPlayer::start(VideoContext* ctx, int outWidth, int outHeight, bool withAudio, double startTime)
{
	mContext = ctx;
	mOutWidth = outWidth;
	mOutHeight = outHeight;
	mStartTime = startTime;
	mWithAudio = withAudio && hasAudio() && initAudio();

	mThread = std::thread(&MppVideoPlayer::threadProc, this);
}

void MppVideoPlayer::restart()
{
	{
		std::unique_lock<std::mutex> lock(mLock);
		mRestart = true;
	}
	mWake.notify_all();
}

void MppVideoPlayer::threadProc()
{
	while (!mStop)
	{
		playOnce();

		std::unique_lock<std::mutex> lock(mLock);
		if (mStop)
			break;

		mEnded = true;
		mWake.wait(lock, [this] { return mStop || mRestart; });
		if (mStop)
			break;

		mRestart = false;
		mEnded = false;
		mStartTime = 0;
		lock.unlock();

		rewind();
	}
}

bool MppVideoPlayer::playOnce()
{
	if (mStartTime > 0)
		av_seek_frame(mFormat, -1, (int64_t)(mStartTime * AV_TIME_BASE), AVSEEK_FLAG_BACKWARD);

	AVPacket* out = av_packet_alloc();

	while (!mStop && av_read_frame(mFormat, mPacket) >= 0)
	{
		if (mPacket->stream_index == mVideoStream)
		{
			if (av_bsf_send_packet(mBsf, mPacket) == 0)
			{
				while (!mStop && av_bsf_receive_packet(mBsf, out) == 0)
				{
					sendVideoPacket(out, false);
					av_packet_unref(out);
				}
			}
		}
		else if (mWithAudio && mPacket->stream_index == mAudioStream)
			decodeAudio(mPacket);

		av_packet_unref(mPacket);
	}

	av_packet_free(&out);

	if (mStop)
		return false;

	// flush: everything still queued in the decoder is shown before the end
	sendVideoPacket(nullptr, true);
	drainFrames(true);
	return true;
}

bool MppVideoPlayer::sendVideoPacket(AVPacket* pkt, bool eos)
{
	MppCtx ctx = (MppCtx)mMpp;
	MppApi* mpi = (MppApi*)mMpi;

	MppPacket packet = nullptr;
	if (pkt)
	{
		mpp_packet_init(&packet, pkt->data, pkt->size);
		AVRational tb = mFormat->streams[mVideoStream]->time_base;
		int64_t pts = pkt->pts != AV_NOPTS_VALUE ? pkt->pts : pkt->dts;
		mpp_packet_set_pts(packet, pts != AV_NOPTS_VALUE ? av_rescale_q(pts, tb, AVRational{ 1, 1000000 }) : 0);
	}
	else
		mpp_packet_init(&packet, nullptr, 0);

	if (eos)
		mpp_packet_set_eos(packet);

	// plain memory packets are copied by MPP, so this buffer can be freed afterwards
	bool sent = false;
	while (!mStop)
	{
		if (mpi->decode_put_packet(ctx, packet) == MPP_OK)
		{
			sent = true;
			break;
		}

		// input queue full: take frames out, then retry
		drainFrames(false);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	mpp_packet_deinit(&packet);
	drainFrames(false);
	return sent;
}

bool MppVideoPlayer::drainFrames(bool waitForEos)
{
	MppCtx ctx = (MppCtx)mMpp;
	MppApi* mpi = (MppApi*)mMpi;
	long long deadline = nowMs() + 2000;

	while (!mStop)
	{
		MppFrame frame = nullptr;
		MPP_RET ret = mpi->decode_get_frame(ctx, &frame);

		if (ret != MPP_OK || !frame)
		{
			if (waitForEos && !mFrameEos && nowMs() < deadline)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				continue;
			}
			return mFrameEos;
		}

		if (mpp_frame_get_info_change(frame))
		{
			// first frame (or a resolution change): give the decoder its frame buffers
			if (!mFrameGroup)
			{
				MppBufferGroup group = nullptr;
				// cachable: the NV12 -> RGBA conversion reads every pixel on the CPU
				if (mpp_buffer_group_get_internal(&group, (MppBufferType)(MPP_BUFFER_TYPE_ION | MPP_BUFFER_FLAGS_CACHABLE)) != MPP_OK &&
					mpp_buffer_group_get_internal(&group, MPP_BUFFER_TYPE_ION) != MPP_OK)
				{
					LOG(LogError) << "MppVideoPlayer: no frame buffer group";
					mpp_frame_deinit(&frame);
					mStop = true;
					return false;
				}
				mFrameGroup = group;
			}

			mpp_buffer_group_limit_config((MppBufferGroup)mFrameGroup, mpp_frame_get_buf_size(frame), FRAME_BUFFERS);
			mpi->control(ctx, MPP_DEC_SET_EXT_BUF_GROUP, mFrameGroup);
			mpi->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, nullptr);
		}
		else if (!mpp_frame_get_errinfo(frame) && !mpp_frame_get_discard(frame) && mpp_frame_get_buffer(frame))
			presentFrame(frame);

		if (mpp_frame_get_eos(frame))
			mFrameEos = true;

		mpp_frame_deinit(&frame);
	}

	return mFrameEos;
}

void MppVideoPlayer::presentFrame(void* f)
{
	MppFrame frame = (MppFrame)f;
	long long pts = mpp_frame_get_pts(frame) / 1000; // ms

	if (mClockStart < 0)
	{
		mClockStart = nowMs();
		mFirstPts = pts;
		if (mAudioDevice)
			SDL_PauseAudioDevice(mAudioDevice, 0);
	}

	long long target = mClockStart + (pts - mFirstPts);
	long long now = nowMs();

	if (now > target + LATE_DROP_MS)
		return;

	if (target > now)
	{
		std::unique_lock<std::mutex> lock(mLock);
		mWake.wait_for(lock, std::chrono::milliseconds(target - now), [this] { return mStop.load(); });
		if (mStop)
			return;
	}

	int width = mpp_frame_get_width(frame);
	int height = mpp_frame_get_height(frame);
	int hstride = mpp_frame_get_hor_stride(frame);
	int vstride = mpp_frame_get_ver_stride(frame);
	MppBuffer buffer = mpp_frame_get_buffer(frame);
	unsigned char* base = (unsigned char*)mpp_buffer_get_ptr(buffer);
	if (!base)
		return;

	mSws = sws_getCachedContext(mSws, width, height, AV_PIX_FMT_NV12, mOutWidth, mOutHeight, AV_PIX_FMT_RGBA,
		(width == mOutWidth && height == mOutHeight) ? SWS_POINT : SWS_BILINEAR, nullptr, nullptr, nullptr);
	if (!mSws)
		return;

	const uint8_t* src[2] = { base, base + (size_t)hstride * vstride };
	const int srcStride[2] = { hstride, hstride };

	// same double-buffer protocol as the VLC lock/unlock callbacks
	VideoContext* c = mContext;
	int target_surface = c->surfaceId ^ 1;
	c->mutexes[target_surface].lock();
	c->hasFrame[target_surface] = false;

	uint8_t* dst[1] = { c->surfaces[target_surface] };
	const int dstStride[1] = { mOutWidth * 4 };

	mpp_buffer_sync_begin(buffer);
	sws_scale(mSws, src, srcStride, 0, height, dst, dstStride);
	mpp_buffer_sync_end(buffer);

	c->surfaceId = target_surface;
	c->hasFrame[target_surface] = true;
	c->mutexes[target_surface].unlock();

	if (c->valid && c->component != nullptr && !c->component->isPlaying() && c->component->isWaitingForVideoToStart())
		c->component->onVideoStarted();
}

void MppVideoPlayer::decodeAudio(AVPacket* pkt)
{
	if (!mAudioCodec || avcodec_send_packet(mAudioCodec, pkt) < 0)
		return;

	while (avcodec_receive_frame(mAudioCodec, mAudioFrame) == 0)
	{
		int outSamples = swr_get_out_samples(mSwr, mAudioFrame->nb_samples);
		if (outSamples > 0)
		{
			std::vector<uint8_t> buf((size_t)outSamples * 4);
			uint8_t* out[1] = { buf.data() };
			int n = swr_convert(mSwr, out, outSamples, (const uint8_t**)mAudioFrame->extended_data, mAudioFrame->nb_samples);
			if (n > 0)
				SDL_QueueAudio(mAudioDevice, buf.data(), (Uint32)n * 4);
		}
		av_frame_unref(mAudioFrame);
	}
}

void MppVideoPlayer::rewind()
{
	av_seek_frame(mFormat, mVideoStream, 0, AVSEEK_FLAG_BACKWARD);
	av_bsf_flush(mBsf);
	((MppApi*)mMpi)->reset((MppCtx)mMpp);

	mFrameEos = false;
	mClockStart = -1;

	if (mAudioDevice)
	{
		SDL_PauseAudioDevice(mAudioDevice, 1);
		SDL_ClearQueuedAudio(mAudioDevice);
	}
	if (mAudioCodec)
		avcodec_flush_buffers(mAudioCodec);
	if (mSwr)
		swr_init(mSwr);
}

#endif // HAVE_MPP
