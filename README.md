# fds-utils

Some basic use cases of the Orekit flight dynamics library are covered in this repo.

1 - OEM File generation from TLE

You can modify the input parameters such as TLE, time range, and time step to generate an OEM file you like.

**TODO** The default OEM format is currently set to XML in the Orekit library. Have to add an option for KVN format.

2 - Generate ground station visibilities 

Modify the groundStations.json and tles.txt files to create visibilities for chosen satellite-ground station pairs.
