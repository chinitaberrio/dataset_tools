h264_bag_tools is a c++ program that takes a bag file + h.264 files and replays them as if on a live system. The bag
file must contain frame_info_msg messages that link each frame of the h.264 video files to the bag. The playback
tool publishes topics for the uncorrected images /gmsl/<camera name>/image_color and the corrected images (given the
camera_info_msgs) on /gmsl/<camera name>/image_rect_color.

NOTE: The images are only extracted from the h.264 file if someone has subscribed to the topic. Because the h.264
encoding requires a sequence of frames, all of the images to the current time must be read in sequence. This means that
if you subscribe to an image topic after playback has commenced, the playback will pause while the program reads all of
the images up to the current time. This can be a bit slow if you subscribe to the image topic a long time after playback
starts.

parameters:
    name="output_width" value in pixels
    set the width of the images

    name="output_height" value in pixels
    set the height of the images

    NOTE: if either of the width/height parameters are not defined (or set to 0) the original image size is used

    name="limit_playback_speed" value is boolean
    Either playback the bag + images as fast as possible, or restrict to (close to) realtime playback

    name="bag_file" value is a string
    The name of the bag file to read. The h.264 files and camera names are automatically extracted from the bag file name
    using the first part of the bag file name (without the extension) and the camera name separated using an '_'. Note
    that this is the standard format for the files used in the ACFR campus dataset

example usage:

    roslaunch h264_bag_tools h264_playback.launch bag_file_name:="/home/stew/data/callan-park/2019-04-15-14-37-06_callan_park_loop.bag"

    NOTE: change the output image size in h264_playback.launch
