#!/usr/bin/env python3

import roslibpy

if __name__ == "__main__":
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    print('Is ROS connected? ', client.is_connected)

    listener = roslibpy.Topic(client, '/chatter', 'std_msgs/String')
    listener.subscribe(lambda message: print('Heard talking: ' + message['data']))

    service = roslibpy.Service(client, '/add_two_ints', 'ros_pose_gen/AddTwoInts')
    request = roslibpy.ServiceRequest({'a': 1, 'b': 2})

    #print('Calling service...')
    result = service.call(request)
    print('Service response:  1 + 2 = {}'.format(result['sum']))

    try:
        while True:
            pass
    except KeyboardInterrupt:
        client.terminate()

