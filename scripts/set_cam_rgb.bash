# DEFAULTS
# v4l2-ctl --set-ctrl=brightness=0
# v4l2-ctl --set-ctrl=contrast=28
# v4l2-ctl --set-ctrl=saturation=64
# v4l2-ctl --set-ctrl=hue=0
# v4l2-ctl --set-ctrl=white_balance_automatic=1
# v4l2-ctl --set-ctrl=gamma=100
# v4l2-ctl --set-ctrl=gain=0
# v4l2-ctl --set-ctrl=power_line_frequency=1
# v4l2-ctl --set-ctrl=white_balance_temperature=4600
# v4l2-ctl --set-ctrl=sharpness=3
# v4l2-ctl --set-ctrl=backlight_compensation=16
# v4l2-ctl --set-ctrl=auto_exposure=3
# v4l2-ctl --set-ctrl=exposure_time_absolute=156
# v4l2-ctl --set-ctrl=exposure_dynamic_framerate=0

v4l2-ctl -d /dev/video0 --set-ctrl=brightness=64
v4l2-ctl -d /dev/video0 --set-ctrl=contrast=64
v4l2-ctl -d /dev/video0 --set-ctrl=saturation=0
v4l2-ctl -d /dev/video0 --set-ctrl=hue=0
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_automatic=0
v4l2-ctl -d /dev/video0 --set-ctrl=gamma=85
v4l2-ctl -d /dev/video0 --set-ctrl=gain=100
v4l2-ctl -d /dev/video0 --set-ctrl=power_line_frequency=1
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=2800
v4l2-ctl -d /dev/video0 --set-ctrl=sharpness=6
v4l2-ctl -d /dev/video0 --set-ctrl=backlight_compensation=0
v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=50
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_dynamic_framerate=0

v4l2-ctl -d /dev/video4 --set-ctrl=brightness=64
v4l2-ctl -d /dev/video4 --set-ctrl=contrast=64
v4l2-ctl -d /dev/video4 --set-ctrl=saturation=0
v4l2-ctl -d /dev/video4 --set-ctrl=hue=0
v4l2-ctl -d /dev/video4 --set-ctrl=white_balance_automatic=0
v4l2-ctl -d /dev/video4 --set-ctrl=gamma=85
v4l2-ctl -d /dev/video4 --set-ctrl=gain=100
v4l2-ctl -d /dev/video4 --set-ctrl=power_line_frequency=1
v4l2-ctl -d /dev/video4 --set-ctrl=white_balance_temperature=2800
v4l2-ctl -d /dev/video4 --set-ctrl=sharpness=6
v4l2-ctl -d /dev/video4 --set-ctrl=backlight_compensation=0
v4l2-ctl -d /dev/video4 --set-ctrl=auto_exposure=1
v4l2-ctl -d /dev/video4 --set-ctrl=exposure_time_absolute=50
v4l2-ctl -d /dev/video4 --set-ctrl=exposure_dynamic_framerate=0


#                      brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
#                        contrast 0x00980901 (int)    : min=0 max=64 step=1 default=28 value=28
#                      saturation 0x00980902 (int)    : min=0 max=128 step=1 default=64 value=64
#                             hue 0x00980903 (int)    : min=-40 max=40 step=1 default=0 value=0
#         white_balance_automatic 0x0098090c (bool)   : default=1 value=1
#                           gamma 0x00980910 (int)    : min=72 max=500 step=1 default=100 value=100
#                            gain 0x00980913 (int)    : min=0 max=100 step=1 default=0 value=0
#            power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=1 value=1 (50 Hz)
#       white_balance_temperature 0x0098091a (int)    : min=2800 max=6500 step=1 default=4600 value=4600 flags=inactive
#                       sharpness 0x0098091b (int)    : min=0 max=6 step=1 default=3 value=3
#          backlight_compensation 0x0098091c (int)    : min=0 max=255 step=5 default=16 value=16

# Camera Controls

#                   auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3 (Aperture Priority Mode)
#          exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=156 value=156 flags=inactive
#      exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=0