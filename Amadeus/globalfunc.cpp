#include "stdafx.h"
#include "globalfunc.h"
#include "EBAS_V_UI_R.h"
#include "msql.h"

EBAS_V_UI_RApp theApp;

static void clearnetlist()
{
	for (NetList::iterator i = theApp.m_NetList.begin(); i != theApp.m_NetList.end(); i++)
	{
		for (SiteList::iterator j = i.value()->m_SiteList.begin(); j != i.value()->m_SiteList.end(); j++)
		{
			if (j.value()->psock.state)
			{
				j.value()->close();
			}
			delete j.value();
		}
		i.value()->m_SiteList.clear();
	}
	theApp.m_NetList.clear();
}

void UpdateNetList()
{
	clearnetlist();
	NetList tNetList;
	QString cstr;
	//try
	{
		QStringList   cstrList;

		// ��������
		m_sql.getnetlist(cstrList);
		int size = cstrList.count();
		for (int i = 0; i < size; i++)
		{
			QString NetName = cstrList.at(i);
			NetStr* pNetStr = new NetStr();
			pNetStr->m_runflag = false;
			tNetList[NetName] = pNetStr;

			// ��ȡ������Ϣ�ṹ��
			m_sql.update_netinfo2str(NetName, &pNetStr->m_Netinfo);

			// pNet->m_Serverû�б�ʹ�ã����ڱ��汾�����б�ȥ��
			//if (theApp.m_NetList.find(NetName) != theApp.m_NetList.end())
			//{
			//	pNet->m_Server = theApp.m_NetList.find(NetName)->second->m_Server;
			//}
			//else
			//{
			//	// start server
			//}

			// ��ȡ�����²�վ��Ϣ
			QStringList MountPointList;
			m_sql.getsitelist(NetName, MountPointList);
			int MPsize = MountPointList.count();
			for (int site_i = 0; site_i < MPsize; site_i++)
			{
				cstr = MountPointList.at(site_i);
				QStringList cstrinfo;
				m_sql.getsiteinfo(cstr, cstrinfo);
				if (cstrinfo.count())
					// �ϰ����˴�Ϊcstr = cstrinfo.at(0)���ᵼ����һ��if�����жϵò�����ȷ���
					cstr = cstrinfo.at(1);
				else
					continue;

				if (theApp.m_NetList.find(NetName) != theApp.m_NetList.end()
					&& theApp.m_NetList.find(NetName).value()->m_SiteList.find(cstr)
					!= theApp.m_NetList.find(NetName).value()->m_SiteList.end())
				{
					// theApp���Ѵ��ڣ���ά��theApp�е�ֵ��������theApp
					tNetList[NetName]->m_SiteList[cstr] = theApp.m_NetList.find(NetName).value()->m_SiteList.find(cstr).value();
				}
				// 
				else
				{
					// theApp�в����ڣ���ʹ�����ݿ��еļ�¼����theApp
					CNtripClient tNtripClient;
					//"MOUNTPOINT","SITATABLE","SITETYPE","COM_TYPE","IP","PORT","USERNAME","PASSWORD","LATITUDE","LONGITUDE","HEIGHT"
					int index = 1;
					// ��ȡ����·��,�ϳ���Ϊsprintf(temp->rtcm_fwrt.workpath, pNet->m_Netinfo.WORKPATH, pNet->m_Netinfo.WORKPATH.GetLength());
					sprintf(tNtripClient.rtcm_fwrt.workpath, pNetStr->m_Netinfo.WORKPATH.toUtf8().data());
					// ��ȡ ������������
					tNtripClient.rtcm_fwrt.m_NetName = NetName;
					// ��ȡ ���ص�����
					tNtripClient.MountPoint = cstrinfo.at(index++);
					// ��ȡ ��ʾ��ʶ
					tNtripClient.sitelable = cstrinfo.at(index++);
					// ��ȡ ��վ���� 1 ��׼վ 2 �ƶ�վ 3 �㲥����
					cstr = cstrinfo.at(index++);
					tNtripClient.sitetype = cstr.toInt();
					// ��ȡ ��������  0=TCP CLIENT  1=TCP SERVER  2=NTRIP CLIENT
					cstr = cstrinfo.at(index++);
					tNtripClient.m_ConnectType = (ConnectType)(cstr.toInt());
					// ��ȡ IP��ַ
					tNtripClient.IP = cstrinfo.at(index++);
					// ��ȡ PORT�˿ں�
					tNtripClient.Port = cstrinfo.at(index++);
					// ��ȡ �û���
					tNtripClient.User = cstrinfo.at(index++);
					// ��ȡ ����
					tNtripClient.Pwd = cstrinfo.at(index++);
					// ��ȡ ��վ��γ��
					cstr = cstrinfo.at(index++);
					tNtripClient.XYZ[0] = cstr.toDouble();;
					cstr = cstrinfo.at(index++);
					tNtripClient.XYZ[1] = cstr.toDouble();
					cstr = cstrinfo.at(index++);
					tNtripClient.XYZ[2] = cstr.toDouble();
					ecef2pos(tNtripClient.XYZ, tNtripClient.BLH);
					tNtripClient.netname = NetName;

					// ��tNtripClient���뵽tNetList��SiteList��
					tNetList[tNtripClient.rtcm_fwrt.m_NetName]->m_SiteList.insert(tNtripClient.MountPoint.toUpper(), &tNtripClient);
				}		
			}
		}
	}
	//catch (CDBException* e)
	//{
	//	cstr.Format("ERROR: void CMainFrame::UpdateNetList(void)   %s ", e->m_strError);
	//}

	// ����ԭ�б�ɾ���б����Ĳ�վ
	NetList::iterator Net_i(theApp.m_NetList.begin());
	while (Net_i != theApp.m_NetList.end())
	{
		QMap<QString, CNtripClient*>::iterator site_i(Net_i.value()->m_SiteList.begin());
		while (site_i != Net_i.value()->m_SiteList.end())
		{
			// ������������ʱ�е�ɾ��������ɾ��ԭ�б��ж����(��Ϊ��������ԭ����ֱ��ɾ��)
			if (!(tNetList.find(Net_i.key()) != tNetList.end() && tNetList.find(Net_i.key()).value()->m_SiteList.find(site_i.key()) != tNetList.find(Net_i.key()).value()->m_SiteList.end()))
			{
				CNtripClient* pClient = (CNtripClient*)site_i.value();
				pClient->~CNtripClient();
				site_i.value() = NULL;
			}
			site_i++;
		}
		Net_i++;
	}
	// ɾ��ָ�뱻�ÿյ��б�
	theApp.m_NetList.clear();
	for(Net_i= tNetList.begin(); Net_i != tNetList.end(); Net_i++)
	{ 
		theApp.m_NetList.insert(Net_i.key(), Net_i.value());
	}
}

