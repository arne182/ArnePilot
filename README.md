This README describes the custom features build by me (Arne Schwarck) on top of [ArnePilot](http://github.com/commaai/ArnePilot) of [comma.ai](http://comma.ai). This fork is optimized for the Toyota RAV4 Hybrid 2016 and for driving in Germany but also works with other cars and in other countries. If you would like to support the developement on this project feel free to https://www.patreon.com/arneschwarck
[![](https://i.imgur.com/UelUjKAh.png)](#)

[![demo of ArnePilot with this branch](https://img.youtube.com/vi/WKwSq8TPdpo/0.jpg)](https://www.youtube.com/playlist?list=PL3CGUyxys8DuTE1JTkdZwY93ejSfAGxyV)

For a demo of this version of ArnePilot

Find me on Discord https://discord.gg/Ebgn8Mr

# Installation
Put this URL in the custom URL field after uninstalling through the UI
https://d.sdut.me/arne/release4
or if you want to use the command line or https://github.com/jfrux/workbench
`cd /data; rm -rf openpilot; git clone --depth 1 https://github.com/arne182/openpilot -b release4; reboot`

still have trouble ?? More info about how to install this fork can be found [here](https://medium.com/@jfrux/comma-eon-installing-a-fork-of-openpilot-5c2b5c134b4b).
## Panda flashing

This is done automatically otherwise run (pkill -f boardd; cd /data/openpilot/panda/board; make; reboot) to change the following:
- allowing no disengage on brake and gas for Toyota
- changing acceleration limits for Toyota and
- adapting lane departure warning where it gives you a slight push back into the middle of the lane without needing to be engaged (not yet complete)
- The Panda version is also changed and checked.

## opEdit Demo
<img src=".media/op_edit.gif?raw=true" width="1000">

## Branches

`release4`: this is the default branch that is most up to date with the ArnePilot 0.7 release branch. Normally you should use this branch because it has been tested and verified that it is fully working without any issues.

`075-clean`: this is my old testing branch. I moved on to 077.

`077-clean`: Current development branch

`release3`: this is my old branch, that is compatible with ArnePilot 0.6.

`release2`: this is my old branch, that is compatible with ArnePilot 0.5.

# Configuration

- You can turn on or off some of the [Feature](https://github.com/arne182/ArnePilot/blob/81a3258da49e1bb9739a5f1d0a84b38afd10583f/common/op_params.py#L500) by editing `op_edit.py`. Run the following command `python /data/openpilot/op_edit.py`

# Supported Cars

- RAV4 Hybrid 2016-19
- RAV4 2017-19
- Corolla 2019-20
- Prius 2017
- RX hyrid 2017
- CT 2018
- Hyundia Genesis 2015-16
- Chevrolet Volt 2017

# Todo

- [ ] Finish Dev UI

- [ ] Fix Lag Trafficd

- [ ] get MPC working

- [ ] add APK for OSM and smart steer

# Features

- Braking:
    - by angle(carstate),
    - by predicted angle in 2.5s(laneplanner),
    - by model(commaai),
    - acceleration measured by steering angle,
    - by curvature (mapd),
    - by mapped sign(stop, yield, roundabouts, bump, hump, traffic light, speed sign, road attribute)
- No disengage for gas, only longitudinal disengage for brake, tire slip or cancel
- Only disengage on main off and on brake at low speed
- Smooth longitudinal controller also at low speeds
- No disengage for seat belt remove and door opened. Practical for when stopping and then someone opens a door so that the car does not drive into the lead
- No fingerprint compatibility problems. A completely different way to combine and split Fingerprints so that they always work I.e. comma is not supporting rav4h 2019 because of this Fingerprint method. Mine is better
- Custom events and capnp structure so that comma is happy with the drives from my fork
- Forward collision warning actually brakes for you.
- Ability to ruduce or Increase curvature Factor from `op_edit.py` (`python /data/ArnePilot/op_edit.py`) It will also works with eco and sport mode. If using eco mode then it will start breaking early (350 m before) if using sport mode it will slow down little late (150 m).
- Live speedlimit_offset in `op_tune.py`
- [Dynamic distance profiles](https://github.com/ShaneSmiskol/ArnePilot/tree/stock_additions-devel#dynamic-follow-3-profiles) from Shane (In other word three different dynamic profiles: (`close`, `normal`, `far`, `auto`). Profile can be adjusted from either `python /data/ArnePilot/op_edit.py` or use live tuner to change the profile live (can take up to 4 sec to for new profile to be adjusted) `python /data/ArnePilot/op_tune.py`.
- [Dynamic Gas:](https://github.com/ShaneSmiskol/ArnePilot/tree/stock_additions-devel#dynamic-gas)
This aims to provide a smoother driving experience in stop and go traffic (under 20 mph) by modifying the maximum gas that can be applied based on your current velocity and the relative velocity of the lead car. It'll also of course increase the maximum gas when the lead is accelerating to help you get up to speed quicker than stock. And smoother; this eliminates the jerking you get from stock ArnePilot with comma pedal. It tries to coast if the lead is only moving slowly, it doesn't use maximum gas as soon as the lead inches forward :). When you are above 20 mph, relative velocity and the following distance is taken into consideration.
- We also have enabled commas e2e model which will only work between 11 MPH to 29 MPH. Commas e2e model helps slows down for traffic light, stop sign, etc. e2e, traffic model and mapd all works together to help you stop at the light. All of this can be turned off via `/data/openpilot/op_edit.py`.
- Smart speed (smart speed is essentially speedlimit which eon will not go over unless you have set custom offset) can be overridden by pressing gas above the current smart speed.

# TSS2 Features

- Control 3 gas profiles with sport eco and normal buttons on car
- Blind Spot Monitoring for all of the toyota which will be added to control ALC(vision based lane change from comma.ai). For right now it is always on. It will flash rapidly when stopped and if the object is detected.
- ALC w/ BSM : (Automatic Lane Change with Blind spot monitoring) you can now change lane automataclly. It will wait 1 sec before applying ALC. If the BSM detacts objects it will stop the lane change and will take you back in your original lane. Also, it will notify the user on the eon.
- Reacting Toyota tssp higher acceleration and braking limits.
- Speed sign reading
- Stock Toyota ldw steering assist
- Cruise set speed available down to 7 kph
- Added ability to turn on and off RSA at certain speed. `python /data/openpilot/op_edit.py`

# UI Modifications

- GPS Accurecy on the Dev UI.
- Easily view the EON's IP Address.Just look at the sidebar right under wifi singal strength's.
- Battery has percentage instead of the battery icon.
- If the model detect's cut in it will draw two different chevron to show the user that it see's both of the car.
- Dashcam recording button added to the ui. ( it will save video's to the `/data/media/0/video`)
- Ability to change the SpeedLimit Offset directly from APK. It is based in percentages. For Example, if -1% at 60mph, it will be  approx. 59.4mph, -10% is roughly 54mph etc. (Thank you eFini for the help)
- Dynamic Follow Button: Now you can change the Dynamic Follow Distance just by tapping the blue button on the bottom right. Now with colorful DF button depending on the profile
- Smart speed icon

# Work-in-Progress Features

All WIP features can be configured by modifying `python /data/opepilot/op_edit.py`
- Traffic light detection from Littlemountainman, shane and brain. To get accurate result make sure your daily commute/area is mapped on [OSM](openstreetmap.org) with right direaction. [For example](https://imgur.com/purBVpd)...  [still dont get it watch the video by mageymoo1](https://youtu.be/7dPaF0tDb7Y).
- 2020 Corolla tuning by Spairrow326
- Virtual lane lines and Lane center. This feature is for European roads and is recommended for used in Europe only.
-e2e model button to fix ghost breaking issue. WIP (just need to fix the alerts.)

# Data collection
- Loggin has been Disabled by default on this fork. If you would like to record your drive edit the [following line](https://github.com/arne182/ArnePilot/blob/4d66df96a91c9c13491a3d78b9c1c2a9e848724a/selfdrive/manager.py#L480)
- Offline crash logging. sentry does not catches all the error. now if their is no internet it will still log error in `/data/community/crashes`
- OSM tracers logging and uploading anonymously to help improve MapD as well as OSM accuracy. [Arne is currently ranked 8th for overal tracers uploaded](https://www.openstreetmap.org/stats/data_stats.html).

# Licensing

ArnePilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

<img src="https://d1qb2nb5cznatu.cloudfront.net/startups/i/1061157-bc7e9bf3b246ece7322e6ffe653f6af8-medium_jpg.jpg?buster=1458363130" width="75"></img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>

[![openpilot tests](https://github.com/commaai/openpilot/workflows/openpilot%20tests/badge.svg?event=push)](https://github.com/commaai/openpilot/actions)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/alerts/)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:python)
[![Language grade: C/C++](https://img.shields.io/lgtm/grade/cpp/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:cpp)
[![codecov](https://codecov.io/gh/commaai/openpilot/branch/master/graph/badge.svg)](https://codecov.io/gh/commaai/openpilot)
  
