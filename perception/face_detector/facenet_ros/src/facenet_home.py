from functools import wraps
import numpy as np
import tensorflow as tf
import cv2
import time

import facenet


def timing(f):
    @wraps(f)
    def wrap(*args, **kw):
        ts = time.time()
        result = f(*args, **kw)
        te = time.time()
        print 'func:%r args: took: %2.4f sec' % \
              (f.__name__, te-ts)
        # print 'func:%r args:[%r, %r] took: %2.4f sec' % \
        #   (f.__name__, args, kw, te-ts)
        return result
    return wrap


class Config:
    gpu_memory_fraction = 0.3
    distance_threshold = 0.8

    
class Face:
    def __init__(self, img):
        self.img = img
        self.code = None
        self.name = None


class Recognition:
    def __init__(self, model_path, imgs=None):
        self.init_encoder(model_path)

        if imgs:
            self.init_db(imgs)
            self.identify(self.face_db[0].img)

    @timing
    def init_encoder(self, model_path):
        self.encoder = Encoder(model_path)

    # @timing
    def init_db(self, imgs):
        self.face_db = []
        for img, name in imgs:
            img = cv2.resize(img, (160, 160), interpolation=cv2.INTER_CUBIC)
            face = Face(img)
            face.name = name
            face.code = self.encoder.generate_embedding(img)
            self.face_db.append(face)

    @timing
    def identify(self, face_img):
        face_img = cv2.resize(face_img, (160, 160), interpolation=cv2.INTER_CUBIC)
        face = Face(face_img)
        face.code = self.encoder.generate_embedding(face_img)

        best = 10.0
        for db in self.face_db:
            dist = np.sqrt(np.sum(np.square(np.subtract(face.code, db.code))))
            if dist < best:
                best = dist
                face.name = db.name

        if best < Config.distance_threshold:
            return face
        else:
            return None


class Encoder:
    def __init__(self, model_path):
        config = tf.ConfigProto(
            device_count = {'GPU': 0}
        )
        self.sess = tf.Session(config=config)

        # gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.25)
        # self.sess = tf.Session()
        with self.sess.as_default():
            facenet.load_model(model_path)

    def generate_embedding(self, img):
        # Get input and output tensors
        imgs_placeholder = self.sess.graph.get_tensor_by_name("input:0")
        embeddings = self.sess.graph.get_tensor_by_name("embeddings:0")
        phase_train_placeholder = self.sess.graph.get_tensor_by_name("phase_train:0")

        prewhiten_face = facenet.prewhiten(img)

        # Run forward pass to calculate embeddings
        feed_dict = {imgs_placeholder: [prewhiten_face], phase_train_placeholder: False}
        return self.sess.run(embeddings, feed_dict=feed_dict)[0]
