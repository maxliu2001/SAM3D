class SAM3DError(Exception):

    def __init__(self, message="Error occurred in SAM3D"):
        self.message = message
        super().__init__(self.message)

    def __str__(self):
        return f"SAM3DError: {self.message}"
