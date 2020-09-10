#*****************
# Test pose function
# converting matrix representation
# to cartesian
#*****************

#Set Frames
use_utool 1
use_uframe 3

#get tool frame in matrix representation
pos = indirect('utool',1)
#convert to cartesian format
pose_cnvcart(&pos, 1)