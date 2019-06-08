# husky_agv
Repository WIth all New Husky Nodes

1. Running Husky

* Husky Launches a service called husky-core which starts as soon as you press the button on it
Restart the service-
`sudo service husky-core restart`
* View Service Log
`sudo tail /var/log/upstart/husky-core.log -n 30`
* Location of the launch files the service launches when it starts
`/etc/ros/indigo/husky-core.d`

2. Setup Your Laptop for Husky

*  You can connect to husky via LAN or WiFi. Refer This link to setup your laptop-
`Husky Setup`
* Run the Setup Launch File
`roslaunch setup_husky setup.launch`
   * This script does 2 things
    As the ros::Time on the Husky computer and your laptop isn't the same you it sets the paramater use_sim_time true for the nodes of your laptop to subscribe to /clock(published by husky) instead.
   * Launches a configured rviz
