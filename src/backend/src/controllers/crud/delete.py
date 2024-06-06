import tinydb

db = tinydb.TinyDB("../database/db.json")
User = tinydb.Query()

async def delete(id):
    try:
        db.remove(User.id == id)
        return {"status": "deleted"}
    except Exception as e:
        raise Exception(str(e))
