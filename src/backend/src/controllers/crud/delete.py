import tinydb

db = tinydb.TinyDB("../database/db.json")
User = tinydb.Query()


async def delete(id):
    try:
        db.remove(User.id == id)
        return {"status": "success", "id": id}
    except Exception as e:
        return {"error": str(e)}
