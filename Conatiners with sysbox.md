# The demo shows Docker-in-Docker done securely.

**We show the following actions:**

1.Install the Nestybox container runtime “Sysbox” on the host.
2.Deploy a system container using Docker + Sysbox. The system container has systemd + docker + ssh in its image.
3.Log in to the system container via ssh.
4.Deploy a Docker container inside the system container.
5.Verify the Docker inside the system container is totally isolated from the Docker on the host.
6.Verify that the system container is an unprivileged container.


## Let’s start by installing sysbox
  
``` ls ```

``` sudo dpkg -i sysbox_0.1.2-0.ubuntu-disco_amd64.deb```

Selecting previously unselected pacakage sysbox. 

**Let's deploy our first sys container** 

**We will use a docker image with systemd + docker +ssh in it** 

``` docker run --runtime=sysbox-runc -P -d nestybox/ubuntu-bionic-systemd-docker ```

``` docker ps``` 

**Let's ssh into the image; it's listening on port 32772**

>my host's IP is 10.0.0.230

``` ssh admin@10.0.0.230 -p 32772```

>admin@10.0.0.230's password:                                                                                                            

**To run a command as administrator (user "root"), use "sudo <command>**                                                                  
                                                                                                       
```ps -fu root```

**systemd is init, and it has started docker and ssh ...**

>great feels a bit (or a lot) like a VM ...                                                                            

### let's deploy a container inside the sys container                                                               

```docker ps```                                                                                                        
 
```docker run -it alpine```                                                                                             
                                                                                                                                         
**here we are inside the _inner container_ ...**    


>here we are inside the host 

```docker ps```

>the docker on the host does not see the inner container, it only sees the system container                            

> that's because the docker instance inside the sys container is independent from the one on the host   

**let's exit this inner container**                                                                                                    

#### let's verify this is an unprivileged container                                                                  
 
```cat /proc/self/uid_map  ```                                                                                          
                                                                                                             
>this means the sys container is using the Linux user namespace and                                              

>root in the container is mapped to unprivileged user 231072 on the host                                         

>sysbox assigns exclusive user-ID mappings for each system container, to improve isolation                        

**how about this**                                                                                                  
 
```echo 1 > /proc/sys/kernel/sysrq && echo b > /proc/sysrq-trigger```                                                

>in a privileged container that would have rebooted the host ... not good !                                       

>in a sys container that command fails because root in the container is not root on the host ...               

**let's exit the sys container**                                                                                    

```logout```   
                                                                                                        
##### that's it ...
**For more information on _nestybox/sysbox_  visit [Sysbox quick start guide](https://github.com/nestybox/sysbox/blob/master/docs/quickstart/images.md)**
                                                                                
                                   