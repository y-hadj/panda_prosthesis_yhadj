cd ~/nixpkgs
nix develop

# you are now in an environment with mc_rtc available
# at this point mc_rtc_ticker launches the "official" panda prosthesis controller

cd ~/workspace/panda_prosthesis_rolkneematics
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/home/nixos/install -DMC_RTC_HONOR_INSTALL_PREFIX=ON ..
make -j4
make install


source setup_local.sh

# at this point mc_rtc_ticker launches the "local" panda prosthesis controller
