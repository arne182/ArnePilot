This README describes the custom features build by me (Arne Schwarck) on top of [openpilot](http://github.com/commaai/openpilot) of [comma.ai](http://comma.ai). This fork is optimized for the Toyota RAV4 Hybrid 2016 and for driving in Germany but also works with other cars and in other countries. If you would like to support the developement on this project feel free to https://www.patreon.com/arneschwarck 
- [ ] TODO describe which other cars and countries are known
[![](https://i.imgur.com/UelUjKAh.png)](#)

For a demo of this version of openpilot check the video below:
[![demo of openpilot with this branch](https://img.youtube.com/vi/WKwSq8TPdpo/0.jpg)](https://www.youtube.com/playlist?list=PL3CGUyxys8DuTE1JTkdZwY93ejSfAGxyV)

# Installation
`cd /data; rm -rf openpilot; git clone https://github.com/arne182/openpilot; cd openpilot; git checkout release4; reboot`

still have trouble ?? More info about how to install this fork can be found [here](https://medium.com/@jfrux/comma-eon-installing-a-fork-of-openpilot-5c2b5c134b4b).

## Panda flashing

This is done automatically otherwise run (pkill -f boardd; cd /data/openpilot/panda/board; make; reboot) to change the following:
- allowing no disengage on brake and gas for Toyota
- changing acceleration limits for Toyota and
- adapting lane departure warning where it gives you a slight push back into the middle of the lane without needing to be engaged (not yet complete)
- The Panda version is also changed and checked.

## Branches

`release4`: this is the default branch that is most up to date with the openpilot 0.7 release branch. Normally you should use this branch because it has been tested and verified that it is fully working without any issues.

`073-clean`: this is my default testing branch. When I finishing testing/adding new structure, I'll merge this into the
`release4` branch.

`release3`: this is my old branch, that is compatible with openpilot 0.6.

`release2`: this is my old branch, that is compatible with openpilot 0.5.

# Configuration

- You can turn on or off some of the feature by editing `op_edit.py`. run the following command `python /data/openpilot/op_edit.py`

- You can also use live tuner to edit some of the feature live. `op_tune.py` Run the following command `python /data/openpilot/op_tune.py`

# Todo

- [ ] Auto Lane change from Boggyver on release2 and release3 branch. (only used in released 3 and below)

- [ ] Traffic light detection from Littlemountainman

- [ ] Phantom: control open pilot via app like summon ( only on release 3 and below.)

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
- Reacting Toyota tssp higher acceleration and braking limits.
- Speed sign reading
- Stock Toyota ldw steering assist
- Cruise set speed available down to 7 kph
- Smooth longitudinal controller also at low speeds
- No disengage for seat belt remove and door opened. Practical for when stopping and then someone opens a door so that the car does not drive into the lead
- No fingerprint compatibility problems. A completely different way to combine and split Fingerprints so that they always work I.e. comma is not supporting rav4h 2019 because of this Fingerprint method. Mine is better
- Custom events and capnp structure so that comma is happy with the drives from my fork
- Forward collision warning actually brakes for you.
- Blind Spot Monitoring for all of the toyota which will be added to control ALCA. For right now it is always on. It will flash rapidly when stopped and if the object is detected.
- Ability to ruduce or Increase curvature Factor from `op_edit.py` (`python /data/openpilot/op_edit.py`) It will also works with eco and sport mode. If using eco mode then it will start breaking early (350 m before) if using sport mode it will slow down little late (150 m).
- Ability to change the SpeedLimit Offset directly from APK. It is based in percentages. For Example, if -1% at 60mph, it will be  approx. 59.4mph, -10% is roughly 54mph etc. (Thank you eFini for the help)
- Dashcam recording button added to the ui. ( it will save video's to the `/data/media/0/video`)
- GPS Accurecy on the Dev UI.
- Live speedlimit_offset in op_tune.py
- If the model detect's cut in it will draw two different chevron to show the user that it see's both of the car.
- Control 3 gas profiles with sport eco and normal buttons on car ( only for toyota).
- [Dynamic distance profiles](https://github.com/ShaneSmiskol/openpilot/tree/stock_additions-devel#dynamic-follow-3-profiles) from Shane(In other word three different dynamic profile "Traffic, Relaxed, highway"). Profile can be adjusted from either `python /data/openpilot/op_edit.py` or use live tuner to change the profile live (can take up to 4 sec to for new profile to be adjusted) `python /data/openpilot/op_tune.py`.
- Dynamic Follow Button: Now you can change the Dynamic Follow Distance just but tapping the blue button on the bottom right.
- [Dynamic Gas:](https://github.com/ShaneSmiskol/openpilot/tree/stock_additions-devel#dynamic-gas)
        
        - Currently supported vehicles (w/ comma pedal only):
        
        2017 Toyota Corolla (non-TSS2)
        Toyota RAV4 (non-TSS2)
        2019 Honda Pilot
        TODO: Need to factor in distance, as it will not accelerate to get closer to the stopped lead if you engage at ~0mph far back from the lead.

        This aims to provide a smoother driving experience in stop and go traffic (under 20 mph) by modifying the maximum gas that can be applied based on your current velocity and the relative velocity of the lead car. It'll also of course increase the maximum gas when the lead is accelerating to help you get up to speed quicker than stock. And smoother; this eliminates the jerking you get from stock openpilot with comma pedal. It tries to coast if the lead is only moving slowly, it doesn't use maximum gas as soon as the lead inches forward :). When you are above 20 mph, relative velocity and the following distance is taken into consideration.
- ALC w/ BSM : (Automatic Lane Change with Blind spot monitoring) you can now change lane automataclly. It will wait 1 sec before applying ALC. If the BSM detacts objects it will stop the lane change and will take you back in your original lane. Also, it will notify the user on the eon. 


# Licensing

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

<img src="https://d1qb2nb5cznatu.cloudfront.net/startups/i/1061157-bc7e9bf3b246ece7322e6ffe653f6af8-medium_jpg.jpg?buster=1458363130" width="75"></img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>
