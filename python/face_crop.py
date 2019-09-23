#!usr/bin/env python

from PIL import Image


class FaceCrop:

    image_path = None
    name_image = None
    cpt = 0

    def __init__(self):
        self.name_image = 'imageFlo.jpg'
        self.image_path = 'detected/detected_'

    @staticmethod
    def display_area(area):
        area.show()

    @staticmethod
    def save_area(name_img, number, area):
        new_name = 'cropped/cropped_' + str(number) + '_' + name_img
        area.save(new_name)

    def crop(self, x, y, w, h):
        # Download Image:
        im = Image.open(self.image_path + self.name_image)

        # Check Image Size
        im_size = im.size
        # print(im_size)

        for i in range(len(x)):
            # Define box inside image
            left = x[i]
            top = y[i]
            width = w[i]
            height = h[i]

            # Create Box
            box = (left, top, left + width, top + height)

            # Crop Image
            area = im.crop(box)
            # self.display_area(area)
            # area.show()

            # Save Image
            # print(area.size)
            self.save_area(self.name_image, self.cpt, area)
            # area.save("lena_selected_part.png", "PNG")

#
# if __name__ == '__main__':
#     f = FaceCrop()
