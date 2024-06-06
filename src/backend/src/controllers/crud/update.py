import tinydb

db = tinydb.TinyDB("../database/db.json")
User = tinydb.Query()

async def update(id, version, image, result):
    try:
        db.update({'version': version, 'image': image, 'result': result}, User.id == id)
        return {"id": id, "version": version, "image": image, "result": result}
    except Exception as e:
        raise Exception(str(e))
