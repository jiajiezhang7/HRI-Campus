#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Langchain API 使用示例 - 火山引擎大模型版
这个脚本展示了 langchain 的基本用法，包括:
1. 创建简单的 LLM 链
2. 使用 PromptTemplate
3. 使用对话记忆
4. 使用向量存储进行检索
5. 创建检索增强生成 (RAG) 应用

使用火山引擎大模型API进行测试，而不是OpenAI API
"""

import os
import requests
from langchain.chains import LLMChain
from langchain.prompts import PromptTemplate
from langchain.memory import ConversationBufferMemory
from langchain_core.output_parsers import StrOutputParser
from langchain.llms.base import LLM
from langchain.chains import ConversationalRetrievalChain
from langchain.text_splitter import CharacterTextSplitter
from typing import Any, List, Mapping, Optional
from langchain.callbacks.manager import CallbackManagerForLLMRun


# 创建自定义的火山引擎LLM类
class VolcanoEngineLLM(LLM):
    """火山引擎大模型的LangChain接口"""
    
    api_key: str = ""
    api_url: str = "https://ark.cn-beijing.volces.com/api/v3/chat/completions"
    model_id: str = "ep-20250328110625-qxn6r"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # 获取API密钥，从环境变量获取
        self.api_key = os.environ.get('ARK_API_KEY', '8967f487-1a84-4340-8ac5-79f087456b95')
        self.model_id = os.environ.get('ARK_MODEL_ID', 'ep-20250328110625-qxn6r')
        
    @property
    def _llm_type(self) -> str:
        """返回LLM类型"""
        return "volcano-engine"
    
    def _call(
        self,
        prompt: str,
        stop: Optional[List[str]] = None,
        run_manager: Optional[CallbackManagerForLLMRun] = None,
        **kwargs: Any,
    ) -> str:
        """调用火山引擎大模型API"""
        if not self.api_key:
            raise ValueError("API密钥未设置，无法调用LLM API")
        
        # 准备请求头部
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        
        # 准备消息列表
        messages = [
            {"role": "system", "content": "你是一个有用的AI助手。"},
            {"role": "user", "content": prompt}
        ]
        
        # 准备请求体
        payload = {
            "model": self.model_id,
            "messages": messages
        }
        
        try:
            # 发送请求
            print('正在发送请求到火山引擎大模型API...')
            response = requests.post(self.api_url, headers=headers, json=payload)
            
            # 检查响应
            if response.status_code == 200:
                result = response.json()
                # 根据API返回格式提取文本
                if 'choices' in result and len(result['choices']) > 0:
                    if 'message' in result['choices'][0] and 'content' in result['choices'][0]['message']:
                        response_text = result['choices'][0]['message']['content']
                        print('已收到LLM响应')
                        return response_text
                
                print(f'无法从API响应中提取文本: {result}')
                return "API响应解析错误"
            else:
                print(f'API请求失败: {response.status_code}, {response.text}')
                return f"API请求失败: {response.status_code}"
        
        except Exception as e:
            print(f'调用API时出错: {str(e)}')
            return f"API调用错误: {str(e)}"

def basic_llm_chain_example():
    """基本的 LLM 链示例"""
    print("\n=== 基本的 LLM 链示例 ===")
    
    # 创建提示模板
    template = """
    你是一个有用的AI助手。
    
    用户问题: {question}
    
    请提供一个有帮助的回答:
    """
    prompt = PromptTemplate(template=template, input_variables=["question"])
    
    # 创建火山引擎LLM
    llm = VolcanoEngineLLM()
    
    # 创建 LLM 链
    chain = LLMChain(llm=llm, prompt=prompt)
    
    # 运行链
    response = chain.run(question="什么是人工智能？")
    print(f"回答: {response}")
    
    # 使用新的 Runnable 接口 (推荐的现代方法)
    modern_chain = prompt | llm | StrOutputParser()
    modern_response = modern_chain.invoke({"question": "什么是机器学习？"})
    print(f"现代接口回答: {modern_response}")


def conversation_memory_example():
    """对话记忆示例"""
    print("\n=== 对话记忆示例 ===")
    
    # 创建带有记忆的提示模板
    template = """
    你是一个有用的AI助手。
    
    当前对话历史:
    {chat_history}
    
    用户问题: {question}
    
    请提供一个有帮助的回答:
    """
    prompt = PromptTemplate(
        template=template, 
        input_variables=["chat_history", "question"]
    )
    
    # 创建对话记忆
    memory = ConversationBufferMemory(memory_key="chat_history")
    
    # 创建火山引擎LLM
    llm = VolcanoEngineLLM()
    
    # 创建带有记忆的 LLM 链
    chain = LLMChain(
        llm=llm,
        prompt=prompt,
        memory=memory
    )
    
    # 模拟对话
    print("模拟对话:")
    response1 = chain.run(question="我叫张三，你能记住我的名字吗？")
    print(f"用户: 我叫张三，你能记住我的名字吗？")
    print(f"AI: {response1}")
    
    response2 = chain.run(question="我的名字是什么？")
    print(f"用户: 我的名字是什么？")
    print(f"AI: {response2}")


def retrieval_qa_example():
    """检索问答示例"""
    print("\n=== 检索问答示例 ===")
    print("注意: 这个示例需要额外的依赖，如果失败可能需要安装 sentence-transformers 等包")
    
    # 导入必要的包
    try:
        from langchain_community.embeddings import HuggingFaceEmbeddings
        from langchain_community.vectorstores import FAISS
    except ImportError:
        print("缺少必要的依赖，请安装: pip install sentence-transformers langchain-community")
        return
    
    # 创建示例文档
    documents = [
        "人工智能(AI)是计算机科学的一个分支，旨在创建能够模拟人类智能的系统。",
        "机器学习是人工智能的一个子领域，专注于使用数据来改进性能。",
        "深度学习是机器学习的一个子集，使用神经网络进行学习。",
        "自然语言处理(NLP)是AI的一个分支，专注于使计算机理解和生成人类语言。",
        "计算机视觉是AI的一个领域，专注于使计算机能够从图像或视频中获取信息。"
    ]
    
    # 分割文本
    text_splitter = CharacterTextSplitter(chunk_size=100, chunk_overlap=0)
    texts = text_splitter.create_documents(documents)
    
    # 创建向量存储 (使用本地模型而不是OpenAI)
    try:
        embeddings = HuggingFaceEmbeddings(model_name="sentence-transformers/paraphrase-multilingual-MiniLM-L12-v2")
        vectorstore = FAISS.from_documents(texts, embeddings)
        
        # 创建检索器
        retriever = vectorstore.as_retriever()
        
        # 创建对话记忆
        memory = ConversationBufferMemory(
            memory_key="chat_history",
            return_messages=True
        )
        
        # 创建火山引擎LLM
        llm = VolcanoEngineLLM()
        
        # 创建对话检索链
        qa = ConversationalRetrievalChain.from_llm(
            llm=llm,
            retriever=retriever,
            memory=memory
        )
        
        # 模拟问答
        print("模拟检索问答:")
        query1 = "什么是人工智能？"
        result1 = qa({"question": query1})
        print(f"问题: {query1}")
        print(f"回答: {result1['answer']}")
        
        query2 = "机器学习是什么？它与AI有什么关系？"
        result2 = qa({"question": query2})
        print(f"问题: {query2}")
        print(f"回答: {result2['answer']}")
    
    except Exception as e:
        print(f"执行检索问答示例时出错: {str(e)}")
        print("这可能是由于缺少必要的依赖或模型造成的")


if __name__ == "__main__":
    # 火山引擎 API 密钥已经通过环境变量设置或使用默认值
    # 如果需要，可以在这里手动设置
    # os.environ["ARK_API_KEY"] = "你的火山引擎API密钥"
    # os.environ["ARK_MODEL_ID"] = "ep-20250328110625-qxn6r"
    
    print("Langchain API 使用示例 - 火山引擎大模型版")
    print("使用火山引擎大模型API进行测试")
    
    # 运行示例
    basic_llm_chain_example()
    conversation_memory_example()
    
    # 检索问答示例需要额外的依赖，可能会失败
    try:
        retrieval_qa_example()
    except Exception as e:
        print(f"\n=== 检索问答示例失败 ===")
        print(f"错误: {str(e)}")
        print("注意: 这个示例可能需要安装额外的依赖，如 sentence-transformers")
