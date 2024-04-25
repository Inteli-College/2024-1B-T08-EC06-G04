# Arquitetura de informação

&emsp;&emsp;A **arquitetura da informação** (AI) é uma disciplina do design que se concentra na organização, estrutura e rotulagem de conteúdo em sistemas de informação, como sites, aplicativos móveis e outros espaços digitais. Seu objetivo principal é criar interfaces intuitivas e eficientes que ajudem os usuários a encontrar e interagir com a informação de forma fácil e eficaz.

&emsp;&emsp;A AI é crucial para garantir que os usuários possam navegar de forma eficiente e encontrar o que estão procurando sem dificuldade. Ela aborda questões como a organização de conteúdo, a criação de esquemas de navegação claros, a rotulagem adequada de categorias e a hierarquia de informações.

&emsp;&emsp;A seguir, tem-se duas formas, com perspectivas diferentes, de representar a arquitetura de informação do projeto **Tacomaré**.


## Diagrama de Componentes

&emsp;&emsp;O Diagrama de Componentes é uma das ferramentas fundamentais da Linguagem de Modelagem Unificada (UML), que é amplamente utilizada no campo da engenharia de software para modelar sistemas de software complexos. Especificamente, o Diagrama de Componentes oferece uma representação visual dos componentes de um sistema e das relações entre eles.

&emsp;&emsp;Em termos simples, um componente é uma parte modular e independente de um sistema que encapsula uma funcionalidade específica. Pode ser um módulo de software, uma biblioteca, um serviço web ou qualquer outra unidade lógica que contribua para o funcionamento do sistema como um todo.

&emsp;&emsp;A seguir, tem-se o diagrama de componentes do projeto em questão:

&emsp;&emsp;O que está reprensentado de amarelo (duas celular iniciais superiores), é os dois principais elementos do sistema. Primeiramente, os Robôs TurtleBot, que serão utilizados para portar os sensores de temperatura e a câmera. Outro elemento essencial para o funcionamento do sistema, é o controle do robô, ferramenta utilizada para realizar possíveis movimentos do robô apenas com o controle.

&emsp;&emsp;Sabendo que, o controle será o elemento que mais será utilizado pelos usuários, em verde (núcleo à extrema esqueda da imagem), tem se a parte de visibilidade do sistema com o usuário final. É possível notar no diagrama, que a interface possui os comandos de movimento, onde encontra-se um *joystick* capaz de realizar movimentos de rotação e movimento do Robô TurtleBot. Além disso, na interface terá um feedback para o usuário ter conhecimento do status do sistema (Conectando, conectador, desconectando, desconectado), este elemento entra em conformidade com a primeira Heurística de Nielsen, "Visibilidade do Status do Sistema", que dita que o sistema deve informar ao usuário em qual ambiente ele se encontra, permitindo um maior controle dos utilizadores do sistema.

&emsp;&emsp;Em azul (núcleo maior)