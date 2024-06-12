# SAM Server

## Purpose
This is the server side of SAM service that hosts SAM model and returns requests from client. This component can be scaled on cloud or hosted locally on another machine. The default port is set for `5000`. 

## Terraform Infrastructure Setup
The server has shown to work properly on Lambda Labs GPU instance, but it could also be hosted on AWS/Azure with terraform as infrastructure management tool. The terraform config files has been set up to deploy the server automatically on AWS with a few steps.
### Pre setup
Please create an IAM on your AWS account with FullEC2Access and FullVPCAccess. Then configure your awscli locally:
```
aws config
```
Make sure to enter your IAM ID and secret key. Then, create a keypair on EC2 for the server. A private key file will be downloaded. Generate a public key. 
```
ssh-keygen -t rsa -b 2048 -f my-public-key-path
```
Put the name of the keypair and local path to the generated public key file in `terraform.tfvars` in similar format:
```
key_name = "my-key-pair"
public_key_path = "path/to/your/my-key-pair.pub"
```
The last step is identifying an AMI number and instance type you want to use. If there's no publicly available ones, you may want to create your own (especially for GPU instances). 

In addition to AWS provisioning, a refactored `main-gcp.tf` and `terraform-gcp.tfvars` is included for flexibility to launch the server on GCP. Simply create a GCP project and fill in the corresponding values in the variables file. Only `main.tf` and `terraform.tfvars` will be executed so feel free to rename or delete the redundant files. 

### Create Infra
Simply run
```
terraform init
terraform plan
terraform apply
```
The public IP will be returned as endpoints used by the ROS service. Note that the endpoint could also be ssh connections to other machines. The infra could be destoyed with one line to provide on-demand flexibility for computational cost reduction.
```
terraform destroy
```

### Remote Server on Local GPU Machines
Set up server on local Ubuntu machine by running this code:
```
sudo apt-get update
sudo apt-get install -y python3 python3-pip git
pip3 install git+https://github.com/facebookresearch/segment-anything.git
pip3 install flask
sudo apt-get install -y awscli

git clone https://github.com/maxliu2001/SAM3D.git /home/ubuntu/

cd /home/ubuntu/SAM3D
git checkout server
pip3 install -r requirements.txt

cd /home/ubuntu/SAM3D/SAM_package
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth

python3 api.py
```
Make sure that the port number (5000 by default) traffic is allowed by the firewall. 
```
sudo ufw allow 5000
sudo ufw enable
```
Then, you can format the request url like this on the client side for connection. The server ip can be found by running `ifconfig` or `ipconfig`. Make sure both machines are on the same network. 
```
server_ip = '192.168.1.2'  # Replace with the local machine's IP address
server_port = 5000

response = requests.get(f'http://{server_ip}:{server_port}/')
```
