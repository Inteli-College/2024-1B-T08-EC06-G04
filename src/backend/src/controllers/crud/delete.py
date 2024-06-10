import tinydb

# Lógica para deletar um registro com base no seu ID da base de dados


# Remove da base de dados o registro com id=id
async def delete(id):
    try:
        db = tinydb.TinyDB("../database/db.json")
        User = tinydb.Query()
        db.remove(User.id == id)
        db.close()
        return {"status": "deleted"}
    except Exception as e:
        raise Exception(str(e))
