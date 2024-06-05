import tinydb

db = tinydb.TinyDB("../database/db.json")


async def create(version, image, result):
    try:
        db.insert({"version": version, "image": image, "result": result})
        return {"version": version, "image": image, "result": result}
    except Exception as e:
        return {"error": str(e)}

