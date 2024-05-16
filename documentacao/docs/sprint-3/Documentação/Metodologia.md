---
sidebar_position: 1
custom_edit_url: null
title: "Metodologia"
---

# Introdução

Aqui será apresentada a metodologia do que foi feito durante a terceira Sprint de desenvolvimento da solução com o sistema do Turtlebot, nesta documentação serão abordados os temas relacionados ao sistema `LiDAR` e segurança, além do sistema de aquisição de imagens do robô em tempo real.

:::tip Lembre-se
O `LiDAR` no TurtleBot 3 é um sensor de varredura a laser que mede distâncias até objetos em seu entorno. Ele emite pulsos de laser e calcula o tempo que cada pulso leva para refletir de volta ao sensor, permitindo a criação de um mapa 2D preciso do ambiente.
:::

Segue abaixo as etapas executadas para a realização da terceira Sprint.

## Tecnologias

Durante está sprint, serão utilizadas as seguintes tecnologias;

### 🚀 LiDAR
O LiDAR (Light Detection and Ranging) é um sensor que mede distâncias até objetos em seu entorno. Ele funciona emitindo pulsos de laser e calculando o tempo que cada pulso leva para refletir de volta ao sensor, permitindo a criação de um mapa 2D preciso do ambiente.

**Principais Funções do LiDAR no TurtleBot 3:**

- **Navegação Autônoma:** Ajuda o TurtleBot 3 a navegar de forma autônoma, evitando obstáculos e planejando rotas seguras.
- **Detecção de Obstáculos:** Detecta obstáculos ao redor do robô, permitindo reações a mudanças no ambiente e evitando colisões.

:::note 
O `LiDAR` pode ser utilizado de outras maneiras também. Todavia até então usaremos o mesmo em nossa solução apenas para as funções descritas acima. Se quiser informações adicionais do que o LiDAR é capaz de realizar, recomendo ler este [artigo](https://www.faro.com/en/Resource-Library/Article/What-is-Lidar#:~:text=Lidar%20technology%20is%20an%20ideal,models%20and%20map%20digital%20elevation.).
:::

### 📸 OpenCV
OpenCV (Open Source Computer Vision Library) é uma biblioteca de visão computacional de código aberto que fornece uma ampla gama de ferramentas para processamento de imagem e vídeo.

**Principais Funções do OpenCV:**

- **Processamento de Imagem:** Filtragem, transformação, segmentação e análise das imagens adquiridas pelo robô.
- **Detecção de Objetos:** Algoritmos para reconhecimento e rastreamento de objetos, no caso do projeto será utilizada a detecção de entupimento nos canos.
- **Machine Learning:** Integração com modelos de aprendizado de máquina para tarefas de visão computacional.

### 🎥 Câmera do Dobot Magician
A câmera do Dobot Magician é usada para captura de imagens e vídeos, permitindo a interação visual com o ambiente.

**Principais Funções da Câmera do Dobot Magician:**

- **Captura de Imagem:** Obtenção de imagens estáticas do ambiente para análise.
- **Vídeo em Tempo Real:** Transmissão de vídeo ao vivo para monitoramento e controle.
- **Integração com Visão Computacional:** Utilizada junto com bibliotecas como OpenCV para detecção e reconhecimento de objetos.

### 🌐 Streamlit
Streamlit é uma biblioteca de código aberto para criação de aplicativos web interativos em Python, facilitando a visualização de dados e a criação de interfaces de usuário.

**Principais Funções do Streamlit:**

- **Criação de Interfaces de Usuário:** Ferramentas simples para construir interfaces de usuário interativas.
- **Visualização de Dados:** Integração fácil com bibliotecas de visualização como Matplotlib, Plotly e Altair.
- **Desenvolvimento Rápido:** Facilita o desenvolvimento rápido de protótipos e aplicativos.
- **Interatividade:** Permite a criação de widgets interativos para manipulação de dados em tempo real.

:::info Nota
Como ainda na Sprint 3, o projeto não foi concluído, as tecnologias ainda podem ser mudadas posteriormente.
:::

### Tópicos Utilizados Durante a Sprint 3

Seguindo o mesmo conceito de tópicos citados durante a [metodologia na Sprint 2](../../sprint-2/Documentação/Metodologia.md#tópicos), apresentamos o principal tópico utilizado nesta Sprint para capacitar o robô com a tecnologia `LiDAR`.

#### Tópico Principal

**/scan**: O tópico `scan` desempenha um papel crucial no funcionamento do robô equipado com `LiDAR`. Através deste tópico, o `LiDAR` envia os dados para o sistema ROS (Robot Operating System). Esses dados são indispensáveis para o controle da movimentação do robô, permitindo que ele pare automaticamente ao detectar obstáculos.

:::info Nota
📝 Para mais detalhes sobre a configuração do `LiDAR` e como o tópico `/scan` é implementado no ROS, consulte a [documentação técnica](../../sprint-2/Documentação/Metodologia.md#tópicos-e-nós).
:::

---

## Criação dos Novos Sistemas

A seguir, discutiremos como os novos sistemas de [`LiDAR`](#🚀-lidar) e [processamento de imagens](#📸-opencv) foram integrados ao projeto, juntamente com as principais partes do código relacionadas a esses sistemas.

:::info Nota
Este é um resumo das modificações realizadas. Para acessar o código fonte completo, visite o [GitHub do projeto](https://github.com/Inteli-College/2024-1B-T08-EC06-G04).
:::

### Adição do LiDAR no Código de Movimentação do Robô

Para integrar o `LiDAR` ao código do robô, foram necessárias várias modificações importantes. Entre elas, destacam-se:

- **Escuta do Tópico `scan`**: O robô foi configurado para escutar o tópico `scan` a fim de receber dados de distância fornecidos pelo `LiDAR`.
- **Implementação de Ferramenta de Obstrução**: Foi desenvolvida uma ferramenta que impede a movimentação do robô caso a frente esteja obstruída, evitando que ele avance. De forma similar, se a parte traseira estiver obstruída, o robô não poderá se mover para trás.


### Bibliotecas referentes ao LiDAR utilizadas

Aqui estão as novas bibliotecas que foram integradas ao projeto nesta sprint relacionadas ao LiDAR:

#### LaserScan (sensor_msgs.msg)
A biblioteca `LaserScan` é utilizada para processar os dados adquiridos através da escuta do tópico `scan`. Esta biblioteca é essencial para interpretar as leituras do `LiDAR` e garantir que os dados de distância sejam corretamente compreendidos pelo sistema.

#### qos_profile_sensor_data (rclpy.qos)
A biblioteca `qos_profile_sensor_data` foi usada para adicionar Quality of Service (QOS) ao tópico do ROS2. Isso garante a qualidade na transmissão dos dados sensoriais, permitindo que o robô reaja rapidamente às mudanças no ambiente.

:::info Nota
Todas as bibliotecas apresentadas na [**Sprint 2**](../../sprint-2/Documentação/Metodologia.md#bibliotecas-usadas) continuam sendo utilizadas nesta sprint.
:::

Segue abaixo o import exato das bibliotecas em python:

```python
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
```

:::tip
Nenhuma destas bibliotecas é necessário utilizar o pip para baixar. Ambas **ja fazem parte** do ambiente de execução do ROS2
:::

### Código atualizado referente a movimentação do robô

Abaixo está o código atualizado de movimentação do robô com a integração do LiDAR. As novas funcionalidades introduzidas serão discutidas detalhadamente a seguir, juntamente com a lógica por trás de sua implementação.

:::info Nota
Se seu robô não é capaz de interagir com o LiDAR, é possível seguir utilizando o código desenvolvido na [Sprint 2](https://github.com/Inteli-College/2024-1B-T08-EC06-G04/tree/0.2.0) do robô, porém **limitado** nas funções apresentadas naquela Sprint.
:::

```python
Codigo LIDAR :D
```

### Adição do Streamlit e Processamento de Imagens

Para proporcionar uma melhor visualização do robô, além de um sistema de controle mais otimizado e com uma UX aprimorada, desenvolvemos uma interface de usuário mais intuitiva e direta. Esta nova interface inclui:

- **Visualização em Tempo Real**: Uma câmera acoplada ao robô permite gravar e transmitir em tempo real o que o robô está fazendo, oferecendo uma visão completa das operações.
- **Botões de Controle Intuitivos**: Foram adicionados botões de controle mais intuitivos e fáceis de usar, facilitando a interação e o controle do robô.

A interface foi desenvolvida utilizando o Streamlit, que oferece uma plataforma eficiente para criar aplicações web interativas com Python.

### Bibliotecas Referentes ao Streamlit

Abaixo estão listadas as bibliotecas utilizadas para o funcionamento do Streamlit e para o processamento de imagens em tempo real:

- **cv2**: Utilizada para o processamento de imagens em tempo real.
- **base64**: Utilizada para codificação e decodificação de imagens.
- **numpy (np)**: Utilizada para operações matemáticas e manipulação de arrays.
- **streamlit (st)**: Utilizada para criar a interface web interativa.
- **PIL (Image)**: Utilizada para manipulação de imagens.
- **io**: Utilizada para operações de entrada e saída.

#### Código de Importação

Segue abaixo o código de importação dessas bibliotecas em Python:

```python
import cv2
import base64
import numpy as np
import streamlit as st
from PIL import Image
import io
```

:::info Nota
Exceto pelas bibliotecas `base64` e `io`, todas as outras bibliotecas mencionadas precisam ser instaladas utilizando o gerenciador de pacotes do Python, `pip`. As instruções detalhadas para a instalação estão disponíveis no guia de execução.
:::

### Código referente a interface visual e processamento de imagens

O código abaixo é referente ao processamento das imagens adquiridas pela câmera do robô. Afim de deixar mais claro como o mesmo foi executado e como pode ser alterado afim de abranger todas as necessidades.

:::warning
Qualquer alteração no código pode causar falhas ou mal-funcionamento do robô, altere o mesmo em seu próprio risco.
:::

O código abaixo é utilizado dentro do robô, onde o mesmo serve para passar as informações da webcam para o ambiente do ROS2 que contem o código do Streamlit na máquina pessoal.

```python

# Importações das bibliotecas
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import base64

# Classe principal
class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Erro: Não foi possível acessar a webcam.")
            return
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # Configura a câmera para 60 FPS, se suportado
        timer_period = 0.0167  # aproximadamente 0.0167 segundos (60 FPS)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # Método de callback
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            msg = String()
            msg.data = jpg_as_text
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing image as base64 string')
        else:
            self.get_logger().error('Could not read image from webcam')

    # Método para finalizar a execução da classe
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

:::info Nota
Abaixo será explicado melhor o código, suas classes e métodos mais detalhadamente.
:::

#### Importação das bibliotecas

No específico código acima, a importação para todas as bibliotecas utilizadas é o seguinte:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import base64
```
As bibliotecas referentes a `rclpy` ja são nativas do ambiente de execução do ROS2, ou seja, não precisam ser baixadas com o pip.

#### Classe principal

Segue abaixo a classe principal de execução, os atributos e métodos da mesma.

```python
# Classe principal
class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Erro: Não foi possível acessar a webcam.")
            return
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # Configura a câmera para 60 FPS, se suportado
        timer_period = 0.0167  # aproximadamente 0.0167 segundos (60 FPS)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # Método de callback
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            msg = String()
            msg.data = jpg_as_text
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing image as base64 string')
        else:
            self.get_logger().error('Could not read image from webcam')

    # Método para finalizar a execução da classe
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()
``` 

A classe `Talker` herda de `Node`, a classe base para todos os nós ROS2 em Python. Ela possui os seguintes atributos e métodos:

##### Atributos

- `publisher_`: Publicador ROS que envia mensagens do tipo `String` no tópico `chatter`.
- `cap`: Objeto da classe `cv2.VideoCapture` para captura de vídeo da webcam.
- `timer`: Timer para chamar o método `timer_callback` periodicamente.

##### Métodos

###### `__init__(self)`

- Inicializa a classe, configurando o publicador e a captura de vídeo.
- Configura a taxa de frames por segundo (FPS) da câmera.
- Cria um timer para chamar `timer_callback` a cada 0.033 segundos (30 FPS).

###### `timer_callback(self)`

- Captura uma imagem da webcam.
- Codifica a imagem em base64 e publica como uma mensagem `String`.
- Registra uma mensagem de log indicando que a imagem foi publicada.

###### `destroy_node(self)`

- Libera a captura de vídeo e destrói o nó ROS.

#### Função principal

A função `main()` inicializa o nó ROS, instancia a classe `Talker` e mantém o nó em execução até ser interrompido.

```python
# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Código referente ao recebimento das imagens adquiridas pelo robô e exibição na interface

O código abaixo deve ser executado na máquina atribuída ao robô. O mesmo irá ser responsável por exibir a interface visual ao operador, junto com os controles para operação do robô.

:::warning
Lembrando, deve-se estar conectado na mesma rede __Wi-Fi__ que o robô, caso contrário não irá funcionar.
:::

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import base64
import numpy as np
import streamlit as st
from PIL import Image
import io

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Configuração do Streamlit
        st.title("Visualização da Webcam via ROS 2")
        self.frame_holder = st.empty()

    def listener_callback(self, msg):
        jpg_original = base64.b64decode(msg.data)
        jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)
        img = cv2.imdecode(jpg_as_np, cv2.IMREAD_COLOR)
        if img is not None:
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(img_rgb)
            img_bytes = io.BytesIO()
            pil_image.save(img_bytes, format="JPEG")
            img_bytes.seek(0)
            self.frame_holder.image(
                img_bytes, caption="Webcam Stream", use_column_width=True
            )
        else:
            self.get_logger().error('Could not decode the image')

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Importação das bibliotecas

As bibliotecas usadas nesta parte do código são as seguintes:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import base64
import numpy as np
import streamlit as st
from PIL import Image
import io
```

#### Classe principal

A classe `Listener` herda de `Node`, a classe base para todos os nós ROS2 em Python. Ela possui os seguintes atributos e métodos:

```python
class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Configuração do Streamlit
        st.title("Visualização da Webcam via ROS 2")
        self.frame_holder = st.empty()

    def listener_callback(self, msg):
        jpg_original = base64.b64decode(msg.data)
        jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)
        img = cv2.imdecode(jpg_as_np, cv2.IMREAD_COLOR)
        if img is not None:
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(img_rgb)
            img_bytes = io.BytesIO()
            pil_image.save(img_bytes, format="JPEG")
            img_bytes.seek(0)
            self.frame_holder.image(
                img_bytes, caption="Webcam Stream", use_column_width=True
            )
        else:
            self.get_logger().error('Could not decode the image')
```

#### Atributos

- `subscription`: Inscrição no tópico `chatter` para receber mensagens do tipo `String`.
- `frame_holder`: Elemento de espaço reservado no Streamlit para exibir as imagens.

:::info
Atributos são variáveis que pertencem a um objeto ou classe em programação orientada a objetos. Eles armazenam dados ou informações que são relevantes para o objeto ou classe. No contexto da classe `Listener`, os atributos são usados para manter referências a elementos importantes, como o publicador ROS e o espaço reservado para as imagens no Streamlit.
:::

#### Métodos

##### `__init__(self)`

- Inicializa a classe, configurando a inscrição no tópico e a interface Streamlit.
- Define o título da página Streamlit e cria um espaço vazio para as imagens.

:::info
Métodos são funções definidas dentro de uma classe que descrevem os comportamentos ou ações que um objeto dessa classe pode realizar. Eles podem manipular os atributos do objeto e realizar operações específicas. Na classe `Listener`, os métodos incluem o construtor `__init__`, que inicializa os atributos da classe, e `listener_callback`, que processa as mensagens recebidas e exibe as imagens no Streamlit.
:::

##### `listener_callback(self, msg)`

- Decodifica a imagem base64 recebida.
- Converte a imagem para o formato RGB.
- Exibe a imagem na interface Streamlit.

#### Explicações Adicionais

##### Decodificação da Imagem

- `jpg_original = base64.b64decode(msg.data)`: Decodifica a string base64 em bytes.
- `jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)`: Converte os bytes em um array numpy.
- `img = cv2.imdecode(jpg_as_np, cv2.IMREAD_COLOR)`: Decodifica o array numpy em uma imagem OpenCV.

##### Conversão e Exibição

- `img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)`: Converte a imagem de BGR para RGB.
- `pil_image = Image.fromarray(img_rgb)`: Converte a imagem para o formato PIL.
- `self.frame_holder.image(img_bytes, caption="Webcam Stream", use_column_width=True)`: Exibe a imagem na interface Streamlit.

**Função principal**

A função `main` inicializa o nó ROS, instancia a classe `Listener` e mantém o nó em execução até ser interrompido.

:::warning
Lembre-se de encerrar corretamente o nó ROS2 para evitar problemas de recursos.
:::

## Erros de execução

Afim de prevenir o prosseguimento do código com algum erro ou algo do tipo, todos os códigos realizados possuem um tratamento de erros para que o código seja interrompido, ou pule a etapa se observar algum erro, como pode ser visto a seguir:

No código do processamento e aquisição de imagens

```python
# Trecho do código para tratamento de erros
        else:
            self.get_logger().error('Could not read image from webcam')
```

No código da interface visual e processamento de imagens

```python
# Trecho do código para tratamento de erros
        else:
            self.get_logger().error('Could not decode the image')
```

:::info
Para informações de execução, acesse a documentação a respeito das mesmas.
:::