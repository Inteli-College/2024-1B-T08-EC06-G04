import tinydb

db = tinydb.TinyDB("../database/db.json")
User = tinydb.Query()


async def delete(version):
    try:
        db.remove(User.version == version)
        return {"status": "success", "version": version}
    except Exception as e:
        return {"error": str(e)}
