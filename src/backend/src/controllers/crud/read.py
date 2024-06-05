import tinydb

db = tinydb.TinyDB("../../../../../database/db.json")
User = tinydb.Query()


async def read(version):
    try:
        result = db.search(User.version == version)
        if result:
            return result[0]
        else:
            return {"error": "No record found with the specified version"}
    except Exception as e:
        return {"error": str(e)}

