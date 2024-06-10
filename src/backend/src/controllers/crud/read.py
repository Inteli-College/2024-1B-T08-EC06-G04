import tinydb

db = tinydb.TinyDB("../database/db.json")
User = tinydb.Query()

# Lógica para "ler" as informações de um respectivo id ou, ler todas as informações

async def read(id):
    try:
        result = db.search(User.id == id)
        if result:
            return result[0]
        else:
            return {"error": "No record found with the specified id"}
    except Exception as e:
        return {"error": str(e)}
