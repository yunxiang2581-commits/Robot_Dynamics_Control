# Tailscale + VSCode Remote SSH 配置说明

本文档用于把本机 Ubuntu 开发环境接入 Tailscale，然后在另一台电脑上通过 VSCode Remote SSH 打开本仓库。

## 1. 当前本机 SSH 信息

本机已经确认 OpenSSH Server 可用：

```text
远程用户名: ubuntu
SSH 端口: 22
SSH 服务状态: active
项目路径: /home/ubuntu/Robot_Dynamics_Control
局域网 IP: 192.168.110.93
Tailscale IP: 100.93.3.65
```

本次实际可用的远程连接命令：

```bash
ssh ubuntu@100.93.3.65
```

本次实际可用的 VSCode Remote SSH 配置：

```sshconfig
Host robot-study
    HostName 100.93.3.65
    User ubuntu
    Port 22
```

当前不建议用于外网直连的地址：

```text
10.249.182.128   运营商内网地址，外网通常不能直连 SSH
172.17.0.1       Docker 内部网络
172.18.0.1       Docker 内部网络
198.18.0.1       虚拟/特殊网络地址
```

因此，跨网络远程开发推荐使用：

```text
Tailscale IP + 系统 OpenSSH + VSCode Remote SSH
```

也就是另一台电脑最终连接：

```bash
ssh ubuntu@<本机的 Tailscale IP>
```

其中 `<本机的 Tailscale IP>` 通常形如 `100.x.x.x`。本机当前已经分配到的地址是：

```text
100.93.3.65
```

## 2. 本机 Ubuntu 配置步骤

下面命令需要在本机 Ubuntu 终端执行，因为安装和启动 Tailscale 需要 `sudo` 密码。

### 2.1 安装 Tailscale

推荐使用官方安装脚本：

```bash
curl -fsSL https://tailscale.com/install.sh | sh
```

如果脚本提示需要权限，可使用：

```bash
curl -fsSL https://tailscale.com/install.sh | sudo sh
```

### 2.2 启动服务

```bash
sudo systemctl enable --now tailscaled
```

确认服务状态：

```bash
systemctl is-active tailscaled
```

期望输出：

```text
active
```

### 2.3 登录 Tailscale

```bash
sudo tailscale up --ssh=false
```

说明：

```text
--ssh=false 表示不启用 Tailscale SSH 功能。
本项目仍使用系统 OpenSSH，也就是 ubuntu@<Tailscale IP>:22。
这样和 VSCode Remote SSH 的默认工作方式最一致。
```

执行后终端会输出一个登录链接。打开链接，在浏览器里登录并授权这台 Ubuntu 电脑加入你的 Tailscale 网络。

### 2.4 查看本机 Tailscale IP

```bash
tailscale ip -4
```

记录输出，例如：

```text
100.64.12.34
```

本机当前实际输出为：

```text
100.93.3.65
```

下面文档里用 `<TAILSCALE_IP>` 代替这个地址；按当前配置使用时，可以直接把 `<TAILSCALE_IP>` 替换为 `100.93.3.65`。

### 2.5 验证本机状态

```bash
tailscale status
systemctl is-active ssh
ss -tlnp | grep ':22'
```

期望看到：

```text
tailscale status 能看到本机和另一台电脑
ssh 状态为 active
22 端口处于 LISTEN 状态
```

## 3. 另一台电脑配置步骤

### 3.1 安装并登录 Tailscale

在另一台电脑上安装 Tailscale：

```text
https://tailscale.com/download
```

安装后登录同一个 Tailscale 账号。必须和本机 Ubuntu 在同一个 tailnet 里，否则无法通过 Tailscale IP 互相访问。

### 3.2 测试 SSH

在另一台电脑终端执行：

```bash
ssh ubuntu@<TAILSCALE_IP>
```

本机当前实际命令为：

```bash
ssh ubuntu@100.93.3.65
```

如果第一次连接，会看到主机指纹确认提示。输入：

```text
yes
```

然后输入 Ubuntu 用户 `ubuntu` 的登录密码，或使用你配置好的 SSH 私钥。

### 3.3 配置 VSCode Remote SSH

在另一台电脑安装 VSCode 插件：

```text
Remote - SSH
```

然后编辑本地 SSH 配置文件。

Windows 通常是：

```text
C:\Users\<你的用户名>\.ssh\config
```

Linux / macOS 通常是：

```text
~/.ssh/config
```

加入：

```sshconfig
Host robot-study
    HostName <TAILSCALE_IP>
    User ubuntu
    Port 22
```

本机当前实际配置为：

```sshconfig
Host robot-study
    HostName 100.93.3.65
    User ubuntu
    Port 22
```

如果你使用 SSH 私钥登录，增加一行：

```sshconfig
Host robot-study
    HostName <TAILSCALE_IP>
    User ubuntu
    Port 22
    IdentityFile ~/.ssh/<你的私钥文件>
```

Windows 上 `IdentityFile` 示例：

```sshconfig
Host robot-study
    HostName <TAILSCALE_IP>
    User ubuntu
    Port 22
    IdentityFile C:\Users\<你的用户名>\.ssh\id_ed25519
```

### 3.4 在 VSCode 打开项目

在 VSCode 中执行：

```text
Ctrl+Shift+P
Remote-SSH: Connect to Host...
robot-study
```

连接成功后选择：

```text
Open Folder
/home/ubuntu/Robot_Dynamics_Control
```

## 4. 验证命令

另一台电脑上可以用下面命令逐步检查。

### 4.1 检查 Tailscale 是否能看到本机

```bash
tailscale status
```

应该能看到 Ubuntu 本机设备。

### 4.2 检查 SSH 端口

Windows PowerShell：

```powershell
Test-NetConnection <TAILSCALE_IP> -Port 22
```

期望：

```text
TcpTestSucceeded : True
```

Linux / macOS：

```bash
nc -vz <TAILSCALE_IP> 22
```

期望看到连接成功。

### 4.3 检查项目目录

```bash
ssh ubuntu@<TAILSCALE_IP>
cd /home/ubuntu/Robot_Dynamics_Control
git status
```

## 5. 常见问题

### 5.1 `Connection timed out`

常见含义：网络路径不通。

检查：

```text
1. 两台电脑是否都登录了 Tailscale
2. 是否使用的是 100.x.x.x 形式的 Tailscale IP
3. tailscale status 是否能看到对方设备
4. Ubuntu 本机 tailscaled 是否 active
```

### 5.2 `Connection refused`

常见含义：目标 IP 可达，但目标端口没有服务。

检查 Ubuntu 本机：

```bash
systemctl is-active ssh
ss -tlnp | grep ':22'
```

如果 SSH 没启动：

```bash
sudo systemctl enable --now ssh
```

### 5.3 VSCode 能连 SSH，但打不开项目

检查项目路径是否正确：

```bash
ls /home/ubuntu/Robot_Dynamics_Control
```

本项目路径应为：

```text
/home/ubuntu/Robot_Dynamics_Control
```

### 5.4 不要使用的地址

不要用下面这些地址做跨网络远程 SSH：

```text
10.249.182.128
172.17.0.1
172.18.0.1
198.18.0.1
```

跨网络连接优先使用 Tailscale 分配的：

```text
100.x.x.x
```

## 6. 安全建议

不要公开或发送下面内容：

```text
Ubuntu 登录密码
SSH 私钥内容
Tailscale 登录链接
宽带账号
```

长期使用时，建议后续改成 SSH 密钥登录。第一版可以先用密码验证连通性，确认 VSCode Remote SSH 工作正常后，再逐步切换为密钥。

不要为了远程开发修改 Docker socket 权限，也不要执行：

```bash
chmod 666 /var/run/docker.sock
```
