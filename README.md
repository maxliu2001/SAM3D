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

### Create Infra
Simply run
```
terraform init
terraform plan
terraform apply
```
The public IP will be returned as endpoints used by the ROS service. Note that the endpoint could also be ssh connections to other machines. 
