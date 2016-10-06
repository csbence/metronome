#!/usr/bin/env python3

import json
import numpy as np
import matplotlib.pyplot as plt
from pymongo import MongoClient

import sys

__author__ = 'Bence Cserna'

def open_connection():
    client = MongoClient('mongodb://localhost:27017')
    client.rts.authenticate('rtsUser', 'VeLuvh4!', mechanism='SCRAM-SHA-1')
    return client.rts


def upload_result(db, result):
    transaction_result = db.experimentalResult.insert_one(result)
    pass


def upload_results(db, results):
    db.experimentalResult.insert_many(results)
    pass


def get_results(algorithm, domain, termination_type):
    pass

