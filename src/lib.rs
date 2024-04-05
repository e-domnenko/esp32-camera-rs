#![crate_name = "esp32_camera_rs"]
#![feature(associated_type_bounds)]

use esp_idf_hal::{gpio::Pin, units::MegaHertz};
use esp_idf_sys::{esp, esp_idf_camera as camera, EspError};
use std::marker::PhantomData;

/// Pixel formats supported by ESP32 camera
pub enum PixelFormat {
    RAW,
    GRAYSCALE,
    JPEG,
    RGB444,
    RGB555,
    RGB565,
    RGB888,
    YUV422,
    YUV420
}

impl Into<camera::pixformat_t> for PixelFormat {
    fn into(self) -> camera::pixformat_t {
        match self {
            Self::RAW => camera::pixformat_t_PIXFORMAT_RAW,
            Self::GRAYSCALE => camera::pixformat_t_PIXFORMAT_GRAYSCALE,
            Self::JPEG => camera::pixformat_t_PIXFORMAT_JPEG,
            Self::RGB444 => camera::pixformat_t_PIXFORMAT_RGB444,
            Self::RGB555 => camera::pixformat_t_PIXFORMAT_RGB555,
            Self::RGB565 => camera::pixformat_t_PIXFORMAT_RGB565,
            Self::RGB888 => camera::pixformat_t_PIXFORMAT_RGB888,
            Self::YUV422 => camera::pixformat_t_PIXFORMAT_YUV422,
            Self::YUV420 => camera::pixformat_t_PIXFORMAT_YUV420,
        }
    }
}

impl From<camera::pixformat_t> for PixelFormat {
    fn from(pf: camera::pixformat_t) -> Self {
        match pf {
            camera::pixformat_t_PIXFORMAT_RAW => Self::RAW,
            camera::pixformat_t_PIXFORMAT_GRAYSCALE => Self::GRAYSCALE,
            camera::pixformat_t_PIXFORMAT_JPEG => Self::JPEG,
            camera::pixformat_t_PIXFORMAT_RGB444 => Self::RGB444,
            camera::pixformat_t_PIXFORMAT_RGB555 => Self::RGB555,
            camera::pixformat_t_PIXFORMAT_RGB565 => Self::RGB565,
            camera::pixformat_t_PIXFORMAT_RGB888 => Self::RGB888,
            camera::pixformat_t_PIXFORMAT_YUV422 => Self::YUV422,
            camera::pixformat_t_PIXFORMAT_YUV420 => Self::YUV420,
            _ => unreachable!(),
        }
    }
}

/// Frame sizes supported by ESP32 camera
///
/// *NOTE: Refer to your camera sensor documentation for supported frame sizes*
pub enum FrameSize {
    /// 96x96
    F96x96,
    /// 160x120
    QQVGA,
    /// 176x144
    QCIF,
    /// 240x176
    HQVGA,
    /// 240x240
    F240x240,
    /// 320x240
    QVGA,
    /// 400x296
    CIF,
    /// 480x320
    HVGA,
    /// 640x480
    VGA,
    /// 800x600
    SVGA,
    /// 1024x768
    XGA,
    /// 1280x720
    HD,
    /// 1280x1024
    SXGA,
    /// 1600x1200
    UXGA,
    /// 1920x1080
    FHD,
    /// 720x1280
    P_HD,
    /// 864x1536
    P_3MP,
    /// 2048x1536
    QXGA,
    /// 2560x1440
    QHD,
    /// 2560x1600
    WQXGA,
    /// 1080x1920
    P_FHD,
    /// 2560x1920
    QSXGA,
    Invalid,
}

impl Into<camera::framesize_t> for FrameSize {
    fn into(self) -> camera::framesize_t {
        match self {
            Self::F96x96 => camera::framesize_t_FRAMESIZE_96X96,
            Self::QQVGA => camera::framesize_t_FRAMESIZE_QQVGA,
            Self::QCIF => camera::framesize_t_FRAMESIZE_QCIF,
            Self::HQVGA => camera::framesize_t_FRAMESIZE_HQVGA,
            Self::F240x240 => camera::framesize_t_FRAMESIZE_240X240,
            Self::QVGA => camera::framesize_t_FRAMESIZE_QVGA,
            Self::CIF => camera::framesize_t_FRAMESIZE_CIF,
            Self::HVGA => camera::framesize_t_FRAMESIZE_HVGA,
            Self::VGA => camera::framesize_t_FRAMESIZE_VGA,
            Self::SVGA => camera::framesize_t_FRAMESIZE_SVGA,
            Self::XGA => camera::framesize_t_FRAMESIZE_XGA,
            Self::HD => camera::framesize_t_FRAMESIZE_HD,
            Self::SXGA => camera::framesize_t_FRAMESIZE_SXGA,
            Self::UXGA => camera::framesize_t_FRAMESIZE_UXGA,
            Self::FHD => camera::framesize_t_FRAMESIZE_FHD,
            Self::P_HD => camera::framesize_t_FRAMESIZE_P_HD,
            Self::P_3MP => camera::framesize_t_FRAMESIZE_P_3MP,
            Self::QXGA => camera::framesize_t_FRAMESIZE_QXGA,
            Self::QHD => camera::framesize_t_FRAMESIZE_QHD,
            Self::WQXGA => camera::framesize_t_FRAMESIZE_WQXGA,
            Self::P_FHD => camera::framesize_t_FRAMESIZE_P_FHD,
            Self::QSXGA => camera::framesize_t_FRAMESIZE_QSXGA,
            Self::Invalid => camera::framesize_t_FRAMESIZE_INVALID,
        }
    }
}

/// Location where to allocate frame buffer
pub enum FrameBufferLocation {
    /// Frame buffer is placed in external DRAM
    InDRAM,
    /// Frame buffer is placed in external PSRAM
    InPSRAM,
}

impl Into<camera::camera_fb_location_t> for FrameBufferLocation {
    fn into(self) -> camera::camera_fb_location_t {
        match self {
            Self::InPSRAM => camera::camera_fb_location_t_CAMERA_FB_IN_PSRAM,
            Self::InDRAM => camera::camera_fb_location_t_CAMERA_FB_IN_DRAM,
        }
    }
}

/// Configuration structure for camera initialization
pub enum CameraGrabMode {
    /// Fills buffers when they are empty. Less resources but first 'fb_count' frames might be old
    GrabWhenEmpty,
    /// Except when 1 frame buffer is used, queue will always contain the last 'fb_count' frames
    GrabLatest,
}

impl Into<camera::camera_grab_mode_t> for CameraGrabMode {
    fn into(self) -> camera::camera_grab_mode_t {
        match self {
            Self::GrabWhenEmpty => camera::camera_grab_mode_t_CAMERA_GRAB_WHEN_EMPTY,
            Self::GrabLatest => camera::camera_grab_mode_t_CAMERA_GRAB_LATEST,
        }
    }
}

/// LEDC timer to be used for generating XCLK
pub enum LEDCTimer {
    TIMER0,
    TIMER1,
    TIMER2,
    TIMER3,
}

impl Into<camera::ledc_timer_t> for LEDCTimer {
    fn into(self) -> camera::ledc_timer_t {
        match self {
            Self::TIMER0 => camera::ledc_timer_t_LEDC_TIMER_0,
            Self::TIMER1 => camera::ledc_timer_t_LEDC_TIMER_1,
            Self::TIMER2 => camera::ledc_timer_t_LEDC_TIMER_2,
            Self::TIMER3 => camera::ledc_timer_t_LEDC_TIMER_3,
        }
    }
}

/// LEDC channel to be used for generating XCLK
pub enum LEDCTimerChannel {
    CHANNEL0,
    CHANNEL1,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
    CHANNEL5,
    CHANNEL6,
    CHANNEL7,
}

impl Into<camera::ledc_channel_t> for LEDCTimerChannel {
    fn into(self) -> camera::ledc_channel_t {
        match self {
            Self::CHANNEL0 => camera::ledc_channel_t_LEDC_CHANNEL_0,
            Self::CHANNEL1 => camera::ledc_channel_t_LEDC_CHANNEL_1,
            Self::CHANNEL2 => camera::ledc_channel_t_LEDC_CHANNEL_2,
            Self::CHANNEL3 => camera::ledc_channel_t_LEDC_CHANNEL_3,
            Self::CHANNEL4 => camera::ledc_channel_t_LEDC_CHANNEL_4,
            Self::CHANNEL5 => camera::ledc_channel_t_LEDC_CHANNEL_5,
            Self::CHANNEL6 => camera::ledc_channel_t_LEDC_CHANNEL_6,
            Self::CHANNEL7 => camera::ledc_channel_t_LEDC_CHANNEL_7,
        }
    }
}

/// Configuration structure for camera driver initialization
/// 
/// ### Examples:
/// 
/// #### AI-Thinker board (JPEG, SVGA):
/// 
/// Config {
///     pin_pwdn: Some(peripherals.pins.gpio32),
///     pin_reset: None::<gpio::Gpio0>,
///     pin_xclk: peripherals.pins.gpio0,
///     pin_sccb_sda: peripherals.pins.gpio26,
///     pin_sccb_scl: peripherals.pins.gpio27,
///     pin_d7: peripherals.pins.gpio35,
///     pin_d6: peripherals.pins.gpio34,
///     pin_d5: peripherals.pins.gpio39,
///     pin_d4: peripherals.pins.gpio36,
///     pin_d3: peripherals.pins.gpio21,
///     pin_d2: peripherals.pins.gpio19,
///     pin_d1: peripherals.pins.gpio18,
///     pin_d0: peripherals.pins.gpio5,
///     pin_vsync: peripherals.pins.gpio25,
///     pin_href: peripherals.pins.gpio23,
///     pin_pclk: peripherals.pins.gpio22,
/// 
///     xclk_freq: 20.MHz(),
///     ledc_timer: LEDCTimer::TIMER0,
///     ledc_channel: LEDCTimerChannel::CHANNEL0,
///     pixel_format: PixelFormat::JPEG,
///     frame_size: FrameSize::SVGA,
///     jpeg_quality: 12,
///     fb_count: 1,
///     fb_location: FrameBufferLocation::InPSRAM,
///     grab_mode: CameraGrabMode::GrabWhenEmpty,
/// }
pub struct Config<
    PinPWND: Pin,
    PinReset: Pin,
    PinXCLK: Pin,
    PinSIOD: Pin,
    PinSIOC: Pin,
    PinD7: Pin,
    PinD6: Pin,
    PinD5: Pin,
    PinD4: Pin,
    PinD3: Pin,
    PinD2: Pin,
    PinD1: Pin,
    PinD0: Pin,
    PinVSYNC: Pin,
    PinHREF: Pin,
    PinPCLK: Pin,
> {
    /// GPIO pin for camera power down line
    pub pin_pwdn: Option<PinPWND>,
    /// GPIO pin for camera reset line
    pub pin_reset: Option<PinReset>,
    /// GPIO pin for camera XCLK line
    pub pin_xclk: PinXCLK,
    /// GPIO pin for camera SDA line
    pub pin_sccb_sda: PinSIOD,
    /// GPIO pin for camera SCL line
    pub pin_sccb_scl: PinSIOC,
    /// GPIO pin for camera D7 line
    pub pin_d7: PinD7,
    /// GPIO pin for camera D6 line
    pub pin_d6: PinD6,
    /// GPIO pin for camera D5 line
    pub pin_d5: PinD5,
    /// GPIO pin for camera D4 line
    pub pin_d4: PinD4,
    /// GPIO pin for camera D3 line
    pub pin_d3: PinD3,
    /// GPIO pin for camera D2 line
    pub pin_d2: PinD2,
    /// GPIO pin for camera D1 line
    pub pin_d1: PinD1,
    /// GPIO pin for camera D0 line
    pub pin_d0: PinD0,
    /// GPIO pin for camera VSYNC line
    pub pin_vsync: PinVSYNC,
    /// GPIO pin for camera HREF line
    pub pin_href: PinHREF,
    /// GPIO pin for camera PCLK lin
    pub pin_pclk: PinPCLK,
    /// Frequency of XCLK signal.
    /// 
    /// *EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode*
    pub xclk_freq: MegaHertz,
    /// LEDC timer to be used for generating XCLK
    /// 
    /// *WARNING: May change in next versions tu capture actual timer channel*
    pub ledc_timer: LEDCTimer,
    /// LEDC channel to be used for generating XCLK
    /// 
    /// *WARNING: May change in next versions tu capture actual timer channel*
    pub ledc_channel: LEDCTimerChannel,
    /// Format of the pixel data: YUV422|GRAYSCALE|RGB565|JPEG
    pub pixel_format: PixelFormat,
    /// Size of the output image: QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    pub frame_size: FrameSize,
    /// Quality of JPEG output. 0-63 lower means higher quality
    pub jpeg_quality: i32,
    /// Number of frame buffers to be allocated. If more than one, then each frame will be acquired (double speed)
    pub fb_count: usize,
    /// The location where the frame buffer will be allocated
    pub fb_location: FrameBufferLocation,
    /// When buffers should be filled
    pub grab_mode: CameraGrabMode,
}

impl<
        PinPWND: Pin,
        PinReset: Pin,
        PinXCLK: Pin,
        PinSIOD: Pin,
        PinSIOC: Pin,
        PinD7: Pin,
        PinD6: Pin,
        PinD5: Pin,
        PinD4: Pin,
        PinD3: Pin,
        PinD2: Pin,
        PinD1: Pin,
        PinD0: Pin,
        PinVSYNC: Pin,
        PinHREF: Pin,
        PinPCLK: Pin,
    > Into<camera::camera_config_t>
    for Config<
        PinPWND,
        PinReset,
        PinXCLK,
        PinSIOD,
        PinSIOC,
        PinD7,
        PinD6,
        PinD5,
        PinD4,
        PinD3,
        PinD2,
        PinD1,
        PinD0,
        PinVSYNC,
        PinHREF,
        PinPCLK,
    >
{
    fn into(self) -> camera::camera_config_t {
        camera::camera_config_t {
            pin_pwdn: if let Some(pin) = self.pin_pwdn {
                pin.pin()
            } else {
                -1
            },
            pin_reset: if let Some(pin) = self.pin_reset {
                pin.pin()
            } else {
                -1
            },
            pin_xclk: self.pin_xclk.pin(),
            __bindgen_anon_1: camera::camera_config_t__bindgen_ty_1 {
                pin_sccb_sda: self.pin_sccb_sda.pin(),
            },
            __bindgen_anon_2: camera::camera_config_t__bindgen_ty_2 {
                pin_sccb_scl: self.pin_sccb_scl.pin(),
            },
            pin_d7: self.pin_d7.pin(),
            pin_d6: self.pin_d6.pin(),
            pin_d5: self.pin_d5.pin(),
            pin_d4: self.pin_d4.pin(),
            pin_d3: self.pin_d3.pin(),
            pin_d2: self.pin_d2.pin(),
            pin_d1: self.pin_d1.pin(),
            pin_d0: self.pin_d0.pin(),
            pin_vsync: self.pin_vsync.pin(),
            pin_href: self.pin_href.pin(),
            pin_pclk: self.pin_pclk.pin(),

            xclk_freq_hz: (self.xclk_freq.0 * 1_000_000_u32).try_into().unwrap(),
            ledc_timer: self.ledc_timer.into(),
            ledc_channel: self.ledc_channel.into(),

            pixel_format: self.pixel_format.into(),
            frame_size: self.frame_size.into(),
            jpeg_quality: self.jpeg_quality.into(),
            fb_count: self.fb_count,
            fb_location: self.fb_location.into(),
            grab_mode: self.grab_mode.into(),
            sccb_i2c_port: 0,
        }
    }
}

pub struct FrameBuffer {
    fb: camera::camera_fb_t,
}

impl FrameBuffer {
    pub fn data(&self) -> &mut [u8] {
        unsafe { std::slice::from_raw_parts_mut(self.fb.buf, self.fb.len) }
    }

    pub fn len(&self) -> usize {
        self.fb.len
    }

    pub fn width(&self) -> usize {
        self.fb.width
    }

    pub fn height(&self) -> usize {
        self.fb.height
    }

    pub fn format(&self) -> PixelFormat {
        PixelFormat::from(self.fb.format)
    }

    // TODO: timestamp
}

impl Drop for FrameBuffer {
    fn drop(&mut self) {
        unsafe {
            camera::esp_camera_fb_return(std::ptr::from_mut(&mut self.fb));
        }
    }
}

pub struct EspCamera<
    PinPWND: Pin,
    PinReset: Pin,
    PinXCLK: Pin,
    PinSIOD: Pin,
    PinSIOC: Pin,
    PinD7: Pin,
    PinD6: Pin,
    PinD5: Pin,
    PinD4: Pin,
    PinD3: Pin,
    PinD2: Pin,
    PinD1: Pin,
    PinD0: Pin,
    PinVSYNC: Pin,
    PinHREF: Pin,
    PinPCLK: Pin,
> {
    _p: PhantomData<
        Config<
            PinPWND,
            PinReset,
            PinXCLK,
            PinSIOD,
            PinSIOC,
            PinD7,
            PinD6,
            PinD5,
            PinD4,
            PinD3,
            PinD2,
            PinD1,
            PinD0,
            PinVSYNC,
            PinHREF,
            PinPCLK,
        >,
    >,
}

impl<
        PinPWND: Pin,
        PinReset: Pin,
        PinXCLK: Pin,
        PinSIOD: Pin,
        PinSIOC: Pin,
        PinD7: Pin,
        PinD6: Pin,
        PinD5: Pin,
        PinD4: Pin,
        PinD3: Pin,
        PinD2: Pin,
        PinD1: Pin,
        PinD0: Pin,
        PinVSYNC: Pin,
        PinHREF: Pin,
        PinPCLK: Pin,
    >
    EspCamera<
        PinPWND,
        PinReset,
        PinXCLK,
        PinSIOD,
        PinSIOC,
        PinD7,
        PinD6,
        PinD5,
        PinD4,
        PinD3,
        PinD2,
        PinD1,
        PinD0,
        PinVSYNC,
        PinHREF,
        PinPCLK,
    >
{
    pub fn new(
        config: Config<
            PinPWND,
            PinReset,
            PinXCLK,
            PinSIOD,
            PinSIOC,
            PinD7,
            PinD6,
            PinD5,
            PinD4,
            PinD3,
            PinD2,
            PinD1,
            PinD0,
            PinVSYNC,
            PinHREF,
            PinPCLK,
        >,
    ) -> Result<Self, EspError> {
        esp!(unsafe { camera::esp_camera_init(&config.into()) })?;

        Ok(Self { _p: PhantomData })
    }

    pub fn get_frame_buffer(&self) -> Option<FrameBuffer> {
        unsafe {
            let fb = camera::esp_camera_fb_get();
            if fb.is_null() {
                None
            } else if (*fb).len == 0 || (*fb).buf.is_null() {
                camera::esp_camera_fb_return(fb);
                None
            } else {
                Some(FrameBuffer { fb: *fb })
            }
        }
    }

    // TODO: sensor configuration
}

impl<
        PinPWND: Pin,
        PinReset: Pin,
        PinXCLK: Pin,
        PinSIOD: Pin,
        PinSIOC: Pin,
        PinD7: Pin,
        PinD6: Pin,
        PinD5: Pin,
        PinD4: Pin,
        PinD3: Pin,
        PinD2: Pin,
        PinD1: Pin,
        PinD0: Pin,
        PinVSYNC: Pin,
        PinHREF: Pin,
        PinPCLK: Pin,
    > Drop
    for EspCamera<
        PinPWND,
        PinReset,
        PinXCLK,
        PinSIOD,
        PinSIOC,
        PinD7,
        PinD6,
        PinD5,
        PinD4,
        PinD3,
        PinD2,
        PinD1,
        PinD0,
        PinVSYNC,
        PinHREF,
        PinPCLK,
    >
{
    fn drop(&mut self) {
        esp!(unsafe { camera::esp_camera_deinit() }).expect("error during esp_camera_deinit");
    }
}
