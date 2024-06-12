provider "google" {
  project = var.project_id
  region  = var.region
  zone    = var.zone
}

resource "google_compute_address" "static_ip" {
  name   = "flask-server-ip"
  region = var.region
}

resource "google_compute_network" "vpc_network" {
  name = "flask-vpc-network"
}

resource "google_compute_firewall" "default" {
  name    = "flask-firewall"
  network = google_compute_network.vpc_network.name

  allow {
    protocol = "tcp"
    ports    = ["22", "5000"]
  }

  source_ranges = ["0.0.0.0/0"]
}

resource "google_compute_instance" "flask_server" {
  name         = "flask-server"
  machine_type = "n1-standard-4"
  zone         = var.zone

  boot_disk {
    initialize_params {
      image = "projects/ml-images/global/images/c0-deeplearning-common-cu113-v20211118"
    }
  }

  network_interface {
    network    = google_compute_network.vpc_network.name
    access_config {
      nat_ip = google_compute_address.static_ip.address
    }
  }

  metadata_startup_script = <<-EOF
    #!/bin/bash
    sudo apt-get update
    sudo apt-get install -y python3 python3-pip git
    pip3 install git+https://github.com/facebookresearch/segment-anything.git
    pip3 install flask
    sudo apt-get install -y google-cloud-sdk

    git clone https://github.com/maxliu2001/SAM3D.git /home/ubuntu/

    cd /home/ubuntu/SAM3D
    git checkout server
    pip3 install -r requirements.txt
    
    cd /home/ubuntu/SAM3D/SAM_package
    wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth

    nohup python3 api.py &
    EOF

  tags = ["flask-server"]
}

output "instance_ip" {
  value = google_compute_address.static_ip.address
}