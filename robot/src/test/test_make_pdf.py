#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import fitz

def make_pdf():
    pdf_file = os.path.expanduser("~/Desktop/people.pdf")

    people_path = os.path.expanduser('~/robot_ros/src/robot/image/people')

    image_list = os.listdir(people_path)
    input_file = "./test.pdf"
    file_handle = fitz.open(input_file)

    for image_name in image_list:

        barcode_file = os.path.join(people_path, image_name)

        # define the position (upper-right corner)
        image_rectangle = fitz.Rect(20,20,300,300)

        test_point = fitz.Point(20, 320)
        # retrieve the first page of the PDF

        page = file_handle.newPage()

        # add the image
        page.insertImage(image_rectangle, filename=barcode_file)

        page.insertText(test_point, image_name.split('.')[0])

    file_handle.save(pdf_file)


if __name__ == '__main__':
    make_pdf()