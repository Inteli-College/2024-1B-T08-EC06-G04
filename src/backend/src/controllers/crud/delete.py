import tinydb


async def delete(id):
    try:
        db = tinydb.TinyDB("../database/db.json")
        User = tinydb.Query()
        db.remove(User.id == id)
        db.close()
        return {"status": "deleted"}
    except Exception as e:
        raise Exception(str(e))
