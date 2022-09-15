# AprilTags Example

Sample AprilTags detector and pose estimator.

# Install and Run on Ubuntu

On my Ubuntu workstation, installation is super easy.  First install the pupil-apriltags python wrapper (which also installs apriltags itself):

```
python3 -m pip install pupil-apriltags
```

Then make a fork of github.com/Team100/main2023 into your own account, and then clone it somewhere on your workstation:

```
cd <somewhere handy>
git clone git@github.com:<your account>/main2023.git
cd main2023/apriltags_example
```

To see detections in a single image:

```
python3 pic.py
```

To see detections in your webcam's video stream:

```
python3 vid.py
```

# Install and Run on WPILibPi Raspberry Pi

The Raspberry Pi is what we'll actually be using on the real robot.

Using the WPILibPi raspberry pi image, there are several steps:

First make a fork of github.com/Team100/main2023 into your own account; you'll pull from it below.

Then attach the pi to a monitor, mouse, and keyboard.
Attach the pi to the LAN, e.g. with an ethernet switch.
Turn it on.

The pi boots up in "read only" mode, so we have to change that to allow writing.  From
your laptop browser, browse to http://wpilibpi.local/ and click the "writeable" button on the top.

The attached display should show text. Login using username 'pi' and password 'raspberry'.

First we have to fix the clock, or none of the steps below will work.  Use whatever the actual time is.

```
sudo date -s '15 Sep 2022 14:25'
```

Next we'll add some software.  This takes a long time, like 30 minutes, waiting for the pi to download everything.
From the pi command line, type these:


```
sudo apt update
sudp apt upgrade
sudo apt install lightdm
sudo apt install raspberrypi-ui-mods
sudo apt install git
sudo apt install chromium-browser  (not strictly required but handy!)
python3 -m pip install pupil-apriltags
git clone https://github.com/[your account]/main2023.git
sudo /sbin/reboot now
```

After rebooting, you have to make it "writeable" again, as above: use your laptop to browse to http://wpilibpi.local/ and click the "writeable" button on the top.
We also have to get the WPI stuff to leave the camera alone for now: click "Vision Status" on the left side and click "Down" to stop the vision service.
Click "Vision Settings" on the left and then "remove" if there's a camera listed there.

Now the monitor should show a graphical login screen.  Login with pi/raspberry as before.

Open a terminal window by clicking the icon in the menu bar on the upper left.

```
cd main2023/apriltags
python3 pic.py   (for the canned image)
python3 vid.py   (for the video camera)
```

Point the camera at some AprilTags and rejoice!

# Details

The pose estimator and rendering steps make some assumptions about the camera focal length, which are
surely wrong.
