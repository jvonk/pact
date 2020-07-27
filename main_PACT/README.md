# Data Processing
Install required Python libraries (and Python if you have not already).
```console
pip3 install numpy pandas scipy scikit-learn jupyterlab matplotlib paho-mqtt
```
Make a new file [config.py](config.py) in your editor of choice and populate it with your MQTT username and password as follows
```python
username = "<USERNAME>"
password = "<PASSWORD>"
```
Open the Jupyter notebook of your choice. data_predicting_line.ipynb is the main notebook used for the paper.
```console
jupyter notebook data_predicting_line.ipynb
```
Replace the measured matrix with an array of relative distances between devices. Next, run all the cells in order. The pandas DataFrame structure df should be populated with the MQTT data.

## Data Description

|Filename|Summary|
|-|-|
|pact_20200713T230751.csv|First data collection, different angles and distances, not used in the final models|
|pact_20200704T103931.csv|Part one of data with different angle, positions and 10 devices|
|pact_20200703T210016.csv|Part two of data with different angle, positions and 10 devices, collected immediately after part one|
|pact_20200713T230751.csv|Main data collected with 20 different devices in a straight line, used to find the noise distribution and to measure effects of distance|

### Description of column names
|Column Name|Summary|
|-|-|
|TIMESTAMP|Time data received at computer (went from device to MQTT broker to computer) formatted as YYYY-MM-DD HH:MM:SS.US python format %Y-%m-%d.%H.%M. %S. %f|
|SCANNER|Scanning device name formatted as esp32-# with number ranging from 1 to 20|
|ADVERTISER|Advertising device name formatted as esp32-# with number ranging from 1 to 20|
|TX POWER|RSSI at 1 meter distance from iBeacon packet, matches manafacturer specifications. Formatted as negative integer|
|RSSI|RSSI measurement recorded by scanning device formatted as negative integer|
|DISTANCE|Distance in meters between SCANNER and ADVERTISER formatted as a floating point number|
|ESTIMATE|Estimate of the distance based upon log based model described in paper. 10^((62.235-RSSI_measured)/30.9) Formatted as a floating point number|
|LOG_PREDICTION|Same as ESTIMATE but used in non-linear arrangement of devices|
|ANGLE|Angle of the scanning device relative to the advertising device. Formatted as a floating point number in radians with 0 being directly to the right of the scanner and pi/2 being directly in front, etc.|
|ERROR|ESTIMATE-DISTANCE formatted as floating point number, only used in older, non-linear measurements|