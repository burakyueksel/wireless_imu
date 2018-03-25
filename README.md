## Get your IP Address
$ ifconfig
- Check the results of Ethernet, etc (the most top). Mine is inet addr:192.168.0.95
## Set your App
- Type this address in Wireless IMU software under "Target IP Address"
- Type your desired "Target IP Port".
## Read IMU values from PC and first complementary filter
- sources:
http://www.pieter-jan.com/node/11
- Update "wirelessImuSimpComp.py" accordingly.
$ python wirelessImuSimpComp.py
## Frequencies:
- Fastest 		: 100Hz   	(0.01s)
- Fast (Game) 		: 50Hz 		(0.02s)
- Medium (UI)		: 15Hz  	(0.065)
- Slow (Normal)		: 5Hz     	(0.2s)
## Advanced complementary filter
- sources:
https://github.com/morgil/madgwick_py
http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
https://github.com/superjax/comp_filter
$ python wirelessImuCompFilt.py
- so far it comes nonsense. Need to cross check with the theory...
