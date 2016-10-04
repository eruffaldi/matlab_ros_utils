function m = rosquat2mat(q)


m = ros_pq2mat([0,0,0],q);
