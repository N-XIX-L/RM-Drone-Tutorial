优先保障地面机器人，空中用不起NUC的时候，用upboard也能实现功能，加油打进全国前八经费就充足啦。


F7进BIOS，密码：upassw0rd，或者直接ENTER

安装ubuntu server后，网线直连
```bash
nmap 10.42.0.1/24	//nmap 本机地址/24
```
即可查看upboard IP地址


# 基本配置
系统请参考nuc [RM-Software-Tutorial](https://gdut-dynamic-x.github.io/RM-Software-Tutorial/#/quick_start/installation)，其中配置内核部分未测试用于upboard（可尝试）

所需apt参考 [rm-controls](https://rm-control-docs.netlify.app/en/quick_start/rm-controls_101)



# 开机优化
upboard开机太慢，需要三分多钟，可以修改启动服务占用时间
```bash
systemd-analyze blame

sudo systemctl disable systemd-networkd-wait-online.service
//发现无作用


/*************采用以下方法*************/
sudo vim /usr/lib/systemd/system/systemd-networkd-wait-online.service

//在 [Service] 下的"ExecStart=/lib/systemd/systemd-networkd-online" 后面添加 " --timeout=5"
//开机时间缩短到一分钟
```


# 自启动服务
[rm_bringup](https://github.com/rm-controls/rm_bringup)

```bash
//给普通用户权限，在/etc/sudoers中添加

# For rm_bringup
linuxidc ALL=(ALL:ALL) ALL

//注意：这个不能用echo的方式放在文件最后,必须放在以下两行内容之前
# See sudoers(5) for more information on "#include" directives:
#includedir /etc/sudoers.d



/bin/bash -c "~/rm_start.sh"	//检查是否正常

```


# 进程、线程优化
通过htop查看系统线程和资源配置

PRI(priority)为该线程的优先级，RT(99)为最高优先级，在电脑性能不足时可以把需要的进程优先级设为95(-96)，则该进程下的线程都会共用该优先级。

如何修改程序进程优先级？
[进程优先级](https://linux.cn/article-7325-1.html)