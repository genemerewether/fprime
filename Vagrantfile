# -*- mode: ruby -*-
# vi: set ft=ruby :

VM_NAME = "quest"

Vagrant.configure("2") do |config|  
  # private network is for virtualbox NFS
  config.vm.network "private_network", ip: "192.168.33.10"
  config.vm.network "public_network", bridge: [
    "en0: Wi-Fi (AirPort)",
    "en1: Thunderbolt 1",
    "en2: Thunderbolt 2",
    "eth0"
  ]

  config.vm.synced_folder ".", "/quest-fw", type: "nfs"
  config.vm.synced_folder "~/quest_ws", "/quest_ws", type: "nfs"

  config.vm.box = "ubuntu/xenial64"

  host = RbConfig::CONFIG['host_os']

  # Give VM 1/4 system memory & access to all cpu cores on the host
  if host =~ /darwin/
    cpus = `sysctl -n hw.ncpu`.to_i
    # sysctl returns Bytes and we need to convert to MB
    mem = `sysctl -n hw.memsize`.to_i / 1024 / 1024 / 4
  elsif host =~ /linux/
    cpus = `nproc`.to_i
    # meminfo shows KB and we need to convert to MB
    mem = `grep 'MemTotal' /proc/meminfo | sed -e 's/MemTotal://' -e 's/ kB//'`.to_i / 1024 / 4
  else # sorry Windows folks, I can't help you
    cpus = 2
    mem = 1024
  end

  config.vm.provider "parallels" do |prl, override|
    prl.memory = mem
    prl.cpus = cpus
    prl.customize ["set", :id, "--videosize", "512"]

    override.vm.box = "bento/ubuntu-16.04"

    prl.name = VM_NAME
    prl.update_guest_tools = true
    prl.customize ["set", :id, "--longer-battery-life", "off"]
  end

  config.vm.provider "virtualbox" do |vb|
    vb.gui = false
    vb.customize ["modifyvm", :id, "--ioapic", "on"]

    vb.customize ["modifyvm", :id, "--memory", mem]
    vb.customize ["modifyvm", :id, "--cpus", cpus]

    # Since make and other tools freak out if they see timestamps
    # from the future and we share directories, tightly lock the host and guest clocks together (clock sync if more than 2 seconds off)
    vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-threshold", 2000]
    # Do this on start and restore
    vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-start"]
    vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-on-restore", "1"]
  end

  config.ssh.forward_agent = true
  config.ssh.forward_x11 = true

  config.vm.provision :shell, :path => "provision_vm.bash", privileged: false
  config.vm.provision :shell, :path => "bootstrap.bash", privileged: false
end
