import tinydb

db = tinydb.TinyDB("../../../../../database/db.json")
User = tinydb.Query()


async def update(version, image, result):
    try:
        db.update(
            {"version": version, "image": image, "result": result},
            User.version == version,
        )
        return {"version": version, "image": image, "result": result}
    except Exception as e:
        return {"error": str(e)}
