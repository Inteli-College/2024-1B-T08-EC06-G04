import tinydb

db = tinydb.TinyDB("db.json")
User = tinydb.Query()
db.insert({"version": "v1", "image": 22, "result": False})
print(db.search(User.version == "v1"))
