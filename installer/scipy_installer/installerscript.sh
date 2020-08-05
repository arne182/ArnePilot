#!/data/data/com.termux/files/usr/bin/sh
# Get some needed tools. coreutils for mkdir command, gnugp for the signing key, and apt-transport-https to actually connect to the repo
apt-get update
apt-get --assume-yes upgrade
apt-get --assume-yes install coreutils gnupg
# Make the sources.list.d directory
mkdir -p $PREFIX/etc/apt/sources.list.d
# Write the needed source file
if apt-cache policy | grep -q "termux.*24\|termux.org" ; then
echo "deb https://its-pointless.github.io/files/24 termux extras" > $PREFIX/etc/apt/sources.list.d/pointless.list
else
echo "deb https://its-pointless.github.io/files/21 termux extras" > $PREFIX/etc/apt/sources.list.d/pointless.list
fi
# Add signing key from https://its-pointless.github.io/pointless.gpg
if [ -n $(command -v curl) ]; then
curl -sL https://its-pointless.github.io/pointless.gpg | apt-key add -
elif [ -n $(command -v wget) ]; then
wget -qO - https://its-pointless.github.io/pointless.gpg | apt-key add -
fi
# Update apt
apt update
apt install -y --allow-unauthenticated opencv
apt install -y --allow-unauthenticated scipy
python3 -m pip install overpy
# python3 -m pip install requests
# python3 -m pip install pyzmq
# python3 -m pip install pycapnp
# python3 -m pip install cffi
#mkdir /system/comma/usr/lib/python3.7/site-packages/pyximport
#cp -r /system/comma/usr/lib/python2.7/site-packages/pyximport/. /system/comma/usr/lib/python3.7/site-packages/pyximport
