# standard header:
# - to support the latency metrics of the built-in ROS statistics
# - to hold the sent-counter (i.e. sequence number)
# - to hold the time stamp just before publish is done so the subscriber can calculate the transport latency
Header header
#
# the next resource usage metric is the same you would see in the output of the 'top' command:
# (from the man-page of top)
# VIRT  --  Virtual Memory Size (KiB)
#           The  total  amount  of  virtual  memory  used by the task.  It
#           includes all code, data and shared libraries plus  pages  that
#           have  been swapped out and pages that have been mapped but not
#           used.
# uint64 is a unsigned 64-bit integer (uint64_t in C++)
uint64 virtual_memory_size
# payload is a string of variable length to test various message sizes
string payload
