![kumar's testing_closet](https://github.com/arne182/ArnePilot/workflows/kumar's%20testing_closet/badge.svg?branch=DP08-clean)

This README describes the custom features build by me (Arne Schwarck) on top of [ArnePilot](http://github.com/commaai/ArnePilot) of [comma.ai](http://comma.ai). This fork is optimized for the Toyota RAV4 Hybrid 2016 and Prius TSS2 and for driving in Germany but also works with other cars and in other countries. If you would like to support the developement on this project feel free to https://www.patreon.com/arneschwarck


[![demo of ArnePilot with this branch](https://img.youtube.com/vi/WKwSq8TPdpo/0.jpg)](https://www.youtube.com/playlist?list=PL3CGUyxys8DuTE1JTkdZwY93ejSfAGxyV)

For a demo of this version of ArnePilot

Find me on Discord https://discord.gg/Ebgn8Mr

# Installation

**IF YOU ARE USING DP08X-CLEAN DO NOT TURN ON `ENABLE LOGGER, ENABLE UPLOADER, ENABLE ATHENAD` UNDER DRAGONPILOT APK SETTING. TURNING ON WILL RESULT IN DEVICE BAN.

Put this URL in the custom URL field after uninstalling through the UI
https://tinyurl.com/arne-dp
or if you want to use the command line or https://github.com/jfrux/workbench
`cd /data; rm -rf openpilot; git clone --depth 1 https://github.com/arne182/openpilot -b release5; reboot`

#### Troubleshooting
Arnepilot has comma logger disabled. This gives a 35% more cpu but at cost of giving [connection error](https://cdn.discordapp.com/attachments/538741329799413760/743231854764884067/image0.jpg)

If you get the [no vehicle](https://cdn.discordapp.com/attachments/538741329799413760/743231854764884067/image0.jpg) after installing Arnepilot completely power cycle your device. If this still doesn't fix the problem look below at panda flashing and run the command. This is a known issue with comma2 users.

## Panda flashing
This is done automatically otherwise run `pkill -f boardd; cd /data/openpilot/panda/board; make; ` Make sure this done while your device is connnect to your car:
- allowing no disengage on brake and gas for Toyota
- changing acceleration limits for Toyota and
- adapting lane departure warning where it gives you a slight push back into the middle of the lane without needing to be engaged (not yet complete)
- The Panda version is also changed and checked.

## Branches
`release5`: this is the default branch that is most up to date with the ArnePilot 0.8 based off of [dragonpilot](https://github.com/dragonpilot-community/dragonpilot) release branch. This branch is early in development but will only get better

`DP08-clean`: Current development branch.

`release4`: this is my old branch, that is compatible with ArnePilot 0.7. Recommended branch until release5 gets better

`release3`: this is my old branch, that is compatible with ArnePilot 0.6.

`release2`: this is my old branch, that is compatible with ArnePilot 0.5.


## Supported Cars
Fork is known to work in both US and Europe
- RAV4 Hybrid 2016-19
- RAV4 2017-19
- Corolla 2019-20
- Prius 2017-2021
- RX hyrid 2017
- CT 2018
- Chevrolet Volt 2017
- Subaru Crosstrek Limited 2019 with 0.8 it will use Eyesight for radar. 

### Todo
- [ ] Once QT drops add OSM and Speed offset apk toggles
- [ ] bring back feature op_edit from 0.7
- [ ] Dynamic distance profiles
- [ ] e2e UI button
- [ ] Hands on wheel support
- [ ] Change cruise speed by +- 5MPH

## Features
### Dragonpilot
Since openpilot v0.8.0 Arne has decide to base his fork on [DragonPilot](https://github.com/dragonpilot-community/dragonpilot). So expect all your favorite features to work
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
- Forward collision warning actually brakes for you.
- Smart speed (smart speed is essentially speedlimit which eon will not go over unless you have set custom offset) can be overridden by pressing gas above the current smart speed.
- Hands on wheel sensing to comply with European driving regulations by [alfhern](https://github.com/move-fast)
- Blind Spot Monitoring for all of the toyota which will be added to control ALC(vision based lane change from comma.ai). For right now it is always on. It will flash rapidly when stopped and if the object is detected.
- ALC w/ BSM : (Automatic Lane Change with Blind spot monitoring) you can now change lane automataclly. It will wait 1 sec before applying ALC. If the BSM detacts objects it will stop the lane change and will take you back in your original lane. Also, it will notify the user on the eon.
- Reacting Toyota tssp higher acceleration and braking limits.
- Speed sign reading
- Stock Toyota ldw steering assist
- Cruise set speed available down to 7 kph
- Virtual lane lines and Lane center. This feature is for European roads and is recommended for used in Europe only.
- Alwasys on Dashcam recording ( it will save video's to the `/sdcard/media/dashcam`)

### OpEdit features
all OpEdit features can be manged by running the command `python /data/openpilot/op_edit.py`
- Ability to ruduce or Increase curvature Factor. It will also works with eco and sport mode. If using eco mode then it will start breaking early (350 m before) if using sport mode it will slow down little late (150 m).
- Live speedlimit_offset in `op_tune.py`
- [Dynamic distance profiles](https://github.com/ShaneSmiskol/ArnePilot/tree/stock_additions-devel#dynamic-follow-3-profiles) from Shane (In other word three different dynamic profiles: (`close`, `normal`, `far`, `auto`). Profile can be adjusted from either op_edit.py or using the on screen buttons(can take up to 4 sec to for new profile to be adjusted).
- [Dynamic Gas](https://github.com/ShaneSmiskol/ArnePilot/tree/stock_additions-devel#dynamic-gas)
This aims to provide a smoother driving experience in stop and go traffic (under 20 mph) by modifying the maximum gas that can be applied based on your current velocity and the relative velocity of the lead car. It'll also of course increase the maximum gas when the lead is accelerating to help you get up to speed quicker than stock. And smoother; this eliminates the jerking you get from stock ArnePilot with comma pedal. It tries to coast if the lead is only moving slowly, it doesn't use maximum gas as soon as the lead inches forward :). When you are above 20 mph, relative velocity and the following distance is taken into consideration.
- 2020 Corolla tuning by Spairrow326
- Added ability to turn on and off RSA at certain speeds.
- Control 3 gas profiles with sport eco and normal buttons on car (only on limited car) 
- Ability to change the SpeedLimit Offset directly
- Able to stop using RSA after certain mph. 
- Live indi tune toggle thanks @jamcar23
- cloak mode: which will make comma think you are using their fork. Avoid bans.


### UI Modifications
- Dev UI toggle in APK setting.
- GPS Accurecy on the SideBar
- Battery has percentage instead of the battery icon.
- Dynamic Follow Button
- Smart speed icon
- e2e button

## Data collection
- Loggin has been Disabled by default on this fork. If you would like to record your drive edit the [following line](https://github.com/arne182/ArnePilot/blob/4d66df96a91c9c13491a3d78b9c1c2a9e848724a/selfdrive/manager.py#L480)
- Offline crash logging. sentry does not catches all the error. now if their is no internet it will still log error in `/data/community/crashes`
- OSM tracers logging and uploading anonymously to help improve MapD as well as OSM accuracy. [Arne is currently ranked 8th for overal tracers uploaded](https://www.openstreetmap.org/stats/data_stats.html).
- Added stats that track meter driven as well as overrides/disengagement. These go to a leaderboard. Please added your name to `python /data/opepilot/op_edit.py` to participate.

# Licensing
© OpenStreetMap contributors

ArnePilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

<img src="https://raw.githubusercontent.com/brianandyt/ArnePilot/release4/selfdrive/assets/images/button_home.png" width="75"></img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>
