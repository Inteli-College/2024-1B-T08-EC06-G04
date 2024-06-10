import tinydb

# Lógica para deletar um registro com base no seu ID da base de dados

db = tinydb.TinyDB("../database/db.json")
User = tinydb.Query()

# Remove da base de dados o registro com id=id
async def delete(id):
    try:
        db.remove(User.id == id)
        return {"status": "deleted"}
    except Exception as e:
        raise Exception(str(e))
