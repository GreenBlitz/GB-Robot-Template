import tkinter as tk
from tkinter import PhotoImage

IMAGE_PATH = "images/NoamBatteryMessage.png"
WINDOW_NAME = "Battery Message"


def create_window():
    root = tk.Tk()
    root.title(WINDOW_NAME)
    return root


def load_image(image_path):
    return PhotoImage(file=image_path)


def create_image_label(root, image):
    """Create a label widget to display the image on the given root window."""
    label = tk.Label(root, image=image)
    label.pack()


def terminate_program(root):
    root.quit()


def setup_window():
    """Set up and run the Tkinter event loop."""
    root = create_window()
    image = load_image(IMAGE_PATH)
    create_image_label(root, image)
    root.protocol("WM_DELETE_WINDOW", lambda: terminate_program(root))
    root.mainloop()


if __name__ == "__main__":
    setup_window()
