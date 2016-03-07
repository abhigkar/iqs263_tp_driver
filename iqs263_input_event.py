#!/usr/bin/python
import select
import struct

input_event_path = "/dev/input/event0"

def leftFunc():
    print "LEFT"

def rightFunc():
    print "Right"


def checkEvent(input_event_fd,epoll,touching,left=None,right=None):
    sizeof_input_event = 16
    events = epoll.poll(0)
    for fileno, event in events:
        if event & select.EPOLLIN:
            while True:
                input_event = input_event_fd.read(sizeof_input_event)
                #event = struct.unpack("llHHi",input_event)
                time_sec,time_usec,input_type,input_code,input_value = struct.unpack("2i2Hi",input_event)
                if input_type == 0:
                    break
                if input_code == 330:
                    if input_value == 1:
                        touching[0] = True
                        print "touch:"+str(touching[0])
                    else:
                        touching[0] = False
                        print "touch:"+str(touching[0])
                if input_code == 105 and input_value == 1:
                    if left is not None:
                        left()
                    else:
                        print "left"
                if input_code == 106 and input_value == 1:
                    if right is not None:
                        right()
                    else:
                        print "right"


if __name__ == "__main__":
    input_event_fd = open(input_event_path,'r')
    epoll = select.epoll()
    epoll.register(input_event_fd, select.EPOLLIN)
    touching = [False]
    try:
        while True:
            checkEvent(input_event_fd,epoll,touching,leftFunc,rightFunc)
    except:
        print "except"
    finally:
        epoll.unregister(input_event_fd)
        epoll.close()
        input_event_fd.close()
