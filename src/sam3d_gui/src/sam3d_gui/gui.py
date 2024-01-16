import sys
import cv2
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QHBoxLayout, QListWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
import numpy as np


from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import pyqtSignal, QPoint

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
                # Extract x and y from event.pos(), and set mask to 1
                x, y = event.pos().x(), event.pos().y()
                viewer.addPoint(x, y, 1)
            elif viewer.adding_exc_point:
                # Extract x and y from event.pos(), and set mask to 0 for exclusion point
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

class ImageViewer(QMainWindow):
    def __init__(self, filePath):
        super().__init__()
        self.file_path = filePath
        self.adding_point = False
        self.adding_exc_point = False
        self.adding_edge = False
        self.cv_image = None
        self.original_image = None  # Store the original image
        self.points = []  # List to store points
        self.edges_list = [] # List to store bounding box
        self.initUI()
    
    def initUI(self):
        # Main layout
        main_layout = QHBoxLayout()

        # Image layout with ClickableLabel
        image_layout = QVBoxLayout()
        self.image_label = ClickableLabel(self)
        self.image_label.clicked.connect(self.addPoint)
        self.image_label.boxDrawn.connect(self.drawBoundingBox)
        image_layout.addWidget(self.image_label)
        self.loadImage()
        self.image_label.setAlignment(Qt.AlignCenter)

        # Coordinates layout
        coords_layout = QVBoxLayout()
        self.coords_list = QListWidget()
        coords_layout.addWidget(self.coords_list)

        # Edges layout
        edges_layout = QVBoxLayout()
        self.vertices_list = QListWidget()
        edges_layout.addWidget(self.vertices_list)
        
        # Buttons
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

        # Combine layouts
        main_layout.addLayout(image_layout)
        main_layout.addLayout(coords_layout)
        main_layout.addLayout(edges_layout)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        self.show()

    def loadImage(self):
        self.original_image = cv2.imread(self.file_path)
        self.original_image = cv2.cvtColor(self.original_image, cv2.COLOR_BGR2RGB)
        self.cv_image = self.original_image.copy()
        self.updateImage()

    def updateImage(self):
        if self.cv_image is not None:
            height, width, channel = self.cv_image.shape
            bytesPerLine = 3 * width
            q_img = QImage(self.cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(q_img))
    
    def addPointMode(self):
        # Toggle the adding_point flag
        if self.adding_exc_point:
            self.adding_exc_point = False
        if self.adding_edge:
            self.adding_edge = False
        self.adding_point = not self.adding_point

    def addExcPointMode(self):
        # Toggle the adding_point flag
        if self.adding_point:
            self.adding_point = False
        if self.adding_edge:
            self.adding_edge = False
        self.adding_exc_point = not self.adding_exc_point
    
    def addEdgeMode(self):
        if self.adding_point:
            self.adding_point = False
        if self.adding_exc_point:
            self.adding_exc_point = False
        self.adding_edge = not self.adding_edge
    
    def handle_click(self, x, y):
        if self.adding_point:
            self.addPoint(x, y, 1)
        elif self.adding_exc_point:
            self.addPoint(x, y, 0)

    def addPoint(self, x, y, mask):
        self.points.append([x, y, mask])
        self.coords_list.addItem(f"({x}, {y}, {mask})")
        self.updateImageWithPoints()

    def updateImageWithPoints(self):
        # Redraw the image with all points
        self.cv_image = self.original_image.copy()
        for point in self.points:
            x, y, mask = point
            point = (x,y)
            if mask == 0:
                cv2.circle(self.cv_image, point, 5, (255, 0, 0), -1)
            else:
                cv2.circle(self.cv_image, point, 5, (0, 255, 0), -1)
        self.updateImage()

    def removePoint(self):
        if self.points:
            self.points.pop()
            self.coords_list.takeItem(self.coords_list.count() - 1)
            self.updateImageWithPoints()
        self.updateImage

    def updateImageWithEdges(self):
        self.cv_image = self.original_image.copy()
        for box in self.edges_list:
            cv2.rectangle(self.cv_image, box[0], box[-1], (0, 0, 255), 2)
        self.updateImage()
        
    def removeEdge(self):
        if self.edges_list:
            self.edges_list.pop()
            for _ in range(4):
                if self.vertices_list.count() > 0:
                    self.vertices_list.takeItem(self.vertices_list.count() - 1)
            self.updateImageWithEdges()
    

    def drawBoundingBox(self, startq, endq):
        start = (startq.x(), startq.y())
        end = (endq.x(), endq.y())
        # Calculate box vertices
        top_left = start
        top_right = (end[0], start[1])
        bottom_left = (start[0], end[1])
        bottom_right = end

        # Add vertices to the vertices list
        self.addVertexToList("Top Left", top_left)
        self.addVertexToList("Top Right", top_right)
        self.addVertexToList("Bottom Left", bottom_left)
        self.addVertexToList("Bottom Right", bottom_right)

        self.edges_list.append([start[0], start[1], end[0], end[1]])

        # Draw the box
        self.updateImageWithPoints()  # Redraw points if necessary
        cv2.rectangle(self.cv_image, top_left, bottom_right, (0, 0, 255), 2)
        self.updateImage()

    def addVertexToList(self, vertex_name, point):
        self.vertices_list.addItem(f"{vertex_name}: ({point[0]}, {point[1]})")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ImageViewer('./testimage155.jpg')
    sys.exit(app.exec_())
