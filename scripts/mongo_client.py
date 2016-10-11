#!/usr/bin/env python3

import json
import numpy as np
import re
from pymongo import MongoClient

import sys

__author__ = 'Bence Cserna'


class MetronomeMongoClient:

    def __init__(self):
        self.db = self.open_connection()

    @staticmethod
    def open_connection():
        client = MongoClient('mongodb://localhost:27017')
        client.rts.authenticate('rtsUser', 'VeLuvh4!', mechanism='SCRAM-SHA-1')
        return client.rts

    def upload_result(self, result):
        self.db.experimentalResult.insert_one(result)

    def upload_results(self, results):
        self.db.experimentalResult.insert_many(results)

    def get_results(self, algorithm, domain, instance, termination_type, action_duration):

        query = {
            "experimentConfiguration.algorithmName": algorithm,
            "experimentConfiguration.domainName": domain,
            "experimentConfiguration.domainPath": {"$regex": "%s.*" % re.escape(instance)},
            "experimentConfiguration.terminationType": termination_type,
            "experimentConfiguration.actionDuration": int(action_duration),
            "success": True
        }

        print(query)

        return self.db.experimentalResult.find(query)

