import tinydb

# Lógica da ROTA CREATE

db = tinydb.TinyDB("../database/db.json")

# Função para adquirir próximo ID e aumentar para o próximo CREATE
def get_next_id():
    try:
        max_id = max(item.get('id', 0) for item in db.all())
        return max_id + 1
    except ValueError:
        return 1

# Função para adicionar um novo registro no backend

async def create(version, image, result):
    try:
        id = get_next_id()
        db.insert({"id": id, "version": version, "image": image, "result": result})
        return {"id": id, "version": version, "image": image, "result": result}
    except Exception as e:
        return {"error": str(e)}

