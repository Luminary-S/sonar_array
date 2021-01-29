sonar array
-----

## launch
```
roslaunch sonar_array sonar_array.launch
```
## data polyfit
in data folder, python file is provided for calibrated the sonic distance.

## Details
**hardware**
1. sonic from Jinci tech.  Luo Xiaoliang(wechat)
2. DAQ(data acquisation card to develop) (8AD and 2DA, 24V, 0~10V, 485, 10ms) from YouKong tech

`note: sonic sensor, if not so stable, link a 100uf capacity to the signal and sensor ground wires.`

**DAQ use**
1. init port by sending cmd
2. send cmd to receive data streaming







## Other
test folder for crc check test.