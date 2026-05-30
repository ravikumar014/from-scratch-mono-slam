import cv2

try:
    import sdl2
    import sdl2.ext
    USE_SDL = True
except ImportError:
    print("[INFO] SDL2 not found. Using OpenCV display fallback.")
    USE_SDL = False


class Display(object):
    def __init__(self, W, H):
        self.W, self.H = W, H

        if USE_SDL:
            sdl2.ext.init()
            self.window = sdl2.ext.Window("SLAM", size=(W, H))
            self.window.show()
        else:
            self.window = None  # fallback mode

    def paint(self, img):
        img = cv2.resize(img, (self.W, self.H))

        if USE_SDL:
            events = sdl2.ext.get_events()
            for event in events:
                if event.type == sdl2.SDL_QUIT:
                    exit(0)

            surf = sdl2.ext.pixels3d(self.window.get_surface())
            surf[:, :, 0:3] = img.swapaxes(0, 1)
            self.window.refresh()

        else:
            # 🔥 OpenCV fallback (stable on macOS)
            cv2.imshow("SLAM (Fallback)", img)
            cv2.waitKey(1)