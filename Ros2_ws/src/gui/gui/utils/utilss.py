from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import QByteArray, QBuffer
import numpy as np
import cv2

class Utils:
    @staticmethod
    # Helper function to convert a numpy array image to a QImage
    def arrayToQImage(frame: np.ndarray):
        # image = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (960, 540))

        height, width, channel = image.shape
        bytesPerLine = 3 * width
        # image = cv2.flip(image, 1)
        pic = QImage(
            image.data,
            width,
            height,
            image.strides[0],
            QImage.Format_RGB888,
        )

        return pic

    @staticmethod
    def QImageToArray(pixmap: QPixmap):
        qimage = pixmap.toImage()
        byte_array = QByteArray()
        buffer = QBuffer(byte_array)
        buffer.open(QBuffer.WriteOnly)
        qimage.save(buffer, "PNG")

        np_array = np.frombuffer(byte_array, np.uint8)
        ndarray = cv2.imdecode(np_array, cv2.IMREAD_UNCHANGED)
        return ndarray
