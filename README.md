## INPUT DATA

use the localization result in the submap of Laser T&R

and the submap, and a raw scan of VLP16

## PROCESS

use the kdtree to remove the background, keep the dynamic ojects(people, cars, etc) on.

Cluster on the objects.

## Results

laserScan on the subMap:

![](https://github.com/ZJUYH/test_detection/raw/master/image/1.png)

Use Knn to get the outliers:

![](https://github.com/ZJUYH/test_detection/raw/master/image/2.png)

the cluster results:

![](https://github.com/ZJUYH/test_detection/raw/master/image/0.png)

Process:

![](https://github.com/ZJUYH/test_detection/raw/master/image/4.png)

## PROBLEM

There is problem in the clustering algorithm. 

Maybe the true EMST is really needed.

Many of cluster methods are waiting for me! 

## Thanks

Thanks to Oxford Robotics Institue 

http://www.robots.ox.ac.uk/~mobile/Papers/2012ICRA_wang.pdf

and ETHZ_ASL's libs:

libpointmatcher:   https://github.com/ethz-asl/libpointmatcher

libnabo:   https://github.com/ethz-asl/libnabo
