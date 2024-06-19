import tinydb


# Função que deleta um registro do banco de dados pelo ID
async def delete(id):
    try:
        db = tinydb.TinyDB("../database/db.json")
        User = tinydb.Query()
        db.remove(User.id == id)
        db.close()
        return {"status": "deleted"}
    except Exception as e:
        raise Exception(str(e))
