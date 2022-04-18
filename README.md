# kml_generator

# example

```
roscd kml_generator/data
rosrun kml_generator fixcsv2kml fix.csv fix.kml
```

The fix.csv was generated with the command "rostopic echo -b bag_name.bag -p /topic_name > fix.csv".