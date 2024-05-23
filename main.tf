provider "aws" {
  region = "us-east-2"
}

# AWS key pairs for the project (EC2 key pairs)
resource "aws_key_pair" "deployer" {
  key_name   = var.key_name
  public_key = file(var.public_key_path)
}

resource "aws_security_group" "flask_sg" {
  name        = "flask_security_group"
  description = "Allow HTTP inbound traffic"

  ingress {
    from_port   = 5000
    to_port     = 5000
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
  }

  ingress {
    from_port   = 22
    to_port     = 22
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
  }

  egress {
    from_port   = 0
    to_port     = 0
    protocol    = "-1"
    cidr_blocks = ["0.0.0.0/0"]
  }
}

resource "aws_instance" "flask_server" {
  ami           = "ami-04b70fa74e45c3917" # find aws ami
  instance_type = "g3s.xlarge" # Instance type with GPU
  
  key_name = aws_key_pair.deployer.key_name
  security_groups = [aws_security_group.flask_sg.name]

  user_data = <<-EOF
              #!/bin/bash
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

              nohup python3 api.py &
              EOF

  tags = {
    Name = "FlaskServer"
  }
}

output "instance_ip" {
  value = aws_instance.flask_server.public_ip
}
