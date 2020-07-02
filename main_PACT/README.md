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
Open the Jupyter notebook.
```console
jupyter notebook data_processing.ipynb
```
Replace the measured matrix with an array of relative distances between devices. Next, run all the cells in order. The pandas DataFrame structure df should be populated with the MQTT data.