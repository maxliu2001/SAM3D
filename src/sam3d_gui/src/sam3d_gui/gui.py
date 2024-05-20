from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QLabel, QPushButton, QListWidget, QApplication
from PyQt5.QtGui import QImage, QPixmap, QPainter
from PyQt5.QtCore import Qt, pyqtSignal, QPoint
import numpy as np
import cv2
import sys

class ClickableLabel(QLabel):
    clicked = pyqtSignal(QPoint)  # Signal for point click
    boxDrawn = pyqtSignal(QPoint, QPoint)  # Signal for box drawing

    def __init__(self, parent=None):
        super(ClickableLabel, self).__init__(parent)
        self.setMouseTracking(True)
        self.start_point = None
        self.end_point = None

    def mousePressEvent(self, event):
        viewer = self.getImageViewerParent()
        if viewer:
            if viewer.adding_point:
                x, y = event.pos().x(), event.pos().y()
                viewer.addPoint(x, y, 1)
            elif viewer.adding_exc_point:
                x, y = event.pos().x(), event.pos().y()
                viewer.addPoint(x, y, 0)
            elif viewer.adding_edge:
                self.start_point = event.pos()

    def mouseReleaseEvent(self, event):
        viewer = self.getImageViewerParent()
        if viewer and viewer.adding_edge and self.start_point:
            self.end_point = event.pos()
            self.boxDrawn.emit(self.start_point, self.end_point)
            self.start_point = None
            self.end_point = None

    def getImageViewerParent(self):
        parent = self.parent()
        while parent and not isinstance(parent, ImageViewer):
            parent = parent.parent()
        return parent

    def paintEvent(self, event):
        painter = QPainter(self)
        if self.pixmap():
            painter.drawPixmap(self.rect(), self.pixmap())
        super(ClickableLabel, self).paintEvent(event)

class ImageViewer(QMainWindow):
    def __init__(self, image):
        super().__init__()
        self.adding_point = False
        self.adding_exc_point = False
        self.adding_edge = False
        self.cv_image = image
        self.original_image = image.copy()
        self.points = []
        self.points_masks = []
        self.edges_list = []
        self.initUI()

    def initUI(self):
        print("Initializing UI...")

        self.setFixedSize(800, 600)  # Set fixed size for the window

        main_layout = QHBoxLayout()
        image_layout = QVBoxLayout()
        
        self.image_label = ClickableLabel(self)
        self.image_label.clicked.connect(self.addPoint)
        self.image_label.boxDrawn.connect(self.drawBoundingBox)
        image_layout.addWidget(self.image_label)
        
        self.loadImage()
        self.image_label.setAlignment(Qt.AlignCenter)
        print("Image loaded and added to layout.")

        coords_layout = QVBoxLayout()
        self.coords_list = QListWidget()
        coords_layout.addWidget(self.coords_list)

        edges_layout = QVBoxLayout()
        self.vertices_list = QListWidget()
        edges_layout.addWidget(self.vertices_list)

        add_edge_btn = QPushButton('Add Edges')
        add_edge_btn.clicked.connect(self.addEdgeMode)
        edges_layout.addWidget(add_edge_btn)

        remove_edge_btn = QPushButton('Remove Edges')
        remove_edge_btn.clicked.connect(self.removeEdge)
        edges_layout.addWidget(remove_edge_btn)

        add_btn = QPushButton('Add Valid Point')
        add_btn.clicked.connect(self.addPointMode)
        coords_layout.addWidget(add_btn)

        add_exc_btn = QPushButton('Add Valid Exclusion Point')
        add_exc_btn.clicked.connect(self.addExcPointMode)
        coords_layout.addWidget(add_exc_btn)

        remove_btn = QPushButton('Remove Point')
        remove_btn.clicked.connect(self.removePoint)
        coords_layout.addWidget(remove_btn)

        main_layout.addLayout(image_layout)
        main_layout.addLayout(coords_layout)
        main_layout.addLayout(edges_layout)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        self.show()

        print("UI initialized and displayed.")

    def loadImage(self):
        print("Loading image...")
        self.updateImage()

    def updateImage(self):
        if self.cv_image is not None:
            print("Updating image...")
            height, width, channel = self.cv_image.shape
            bytes_per_line = 3 * width
            cv_image_rgb = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)  # Ensure the image is in RGB format
            q_img = QImage(cv_image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(q_img))
            self.image_label.setFixedSize(width, height)
            print(f"Image updated: {self.cv_image.shape}")
        else:
            print("No image to update.")

    def addPointMode(self):
        print("Toggling add point mode...")
        if self.adding_exc_point:
            self.adding_exc_point = False
        if self.adding_edge:
            self.adding_edge = False
        self.adding_point = not self.adding_point

    def addExcPointMode(self):
        print("Toggling add exclusion point mode...")
        if self.adding_point:
            self.adding_point = False
        if self.adding_edge:
            self.adding_edge = False
        self.adding_exc_point = not self.adding_exc_point

    def addEdgeMode(self):
        print("Toggling add edge mode...")
        if self.adding_point:
            self.adding_point = False
        if self.adding_exc_point:
            self.adding_exc_point = False
        self.adding_edge = not self.adding_edge

    def addPoint(self, x, y, mask):
        print(f"Adding point at ({x}, {y}) with mask {mask}")
        self.points.append([x, y, mask])
        self.coords_list.addItem(f"({x}, {y}, {mask})")
        self.updateImageWithPoints()

    def updateImageWithPoints(self):
        print("Updating image with points...")
        self.cv_image = self.original_image.copy()
        for point in self.points:
            x, y, mask = point
            if mask == 0:
                cv2.circle(self.cv_image, (x, y), 5, (255, 0, 0), -1)
            else:
                cv2.circle(self.cv_image, (x, y), 5, (0, 255, 0), -1)
        self.updateImage()

    def removePoint(self):
        print("Removing last point...")
        if self.points:
            self.points.pop()
            self.coords_list.takeItem(self.coords_list.count() - 1)
            self.updateImageWithPoints()

    def updateImageWithEdges(self):
        print("Updating image with edges...")
        self.cv_image = self.original_image.copy()
        for box in self.edges_list:
            cv2.rectangle(self.cv_image, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 2)
        self.updateImage()

    def removeEdge(self):
        print("Removing last edge...")
        if self.edges_list:
            self.edges_list.pop()
            for _ in range(4):
                if self.vertices_list.count() > 0:
                    self.vertices_list.takeItem(self.vertices_list.count() - 1)
            self.updateImageWithEdges()

    def drawBoundingBox(self, startq, endq):
        print(f"Drawing bounding box from ({startq.x()}, {startq.y()}) to ({endq.x()}, {endq.y()})")
        start = (startq.x(), startq.y())
        end = (endq.x(), endq.y())
        self.edges_list.append([start[0], start[1], end[0], end[1]])
        self.updateImageWithPoints()
        self.updateImageWithEdges()
