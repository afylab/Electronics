#Changelog

6/28/2016: The function bufferRamp() now averages over an specified number of measurements per step to increase the precision in the measurements. The bufferRamp() takes advantage of the adc conversion time by sending the reading from the previous step while waiting for the adc to finish converting.
